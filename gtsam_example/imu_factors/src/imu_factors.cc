#include "imu_factors.hpp"

ImuFactors::ImuFactors()
{
    preint_imu_params_ = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    graph_ = new NonlinearFactorGraph();

    SetImuPreintegrationParams();

    cout << "ImuFactors class instantiation" << endl;
}

ImuFactors::~ImuFactors()
{
    delete(imu_preintegrated_);
}

void ImuFactors::InitImuPreintegration()
{
    /* Parsing input data & extract initial state */
    string value;
    Eigen::Matrix<double, 10, 1> initial_state = Eigen::Matrix<double, 10, 1>::Zero();
    getline(input_file_, value, ',');   // char 'i' filtering
    for(int i=0; i<9; i++){
        getline(input_file_, value, ',');
        initial_state(i) = atof(value.c_str());
    }
    getline(input_file_, value, '\n');
    initial_state(9) = atof(value.c_str());

    /* Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z); */
    Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                          initial_state(4), initial_state(5));
    Point3 prior_point(initial_state.head<3>());
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>());   

    /* Update initial estimated state */
    estimated_state_.insert(X(gps_correction_count_), prior_pose);
    estimated_state_.insert(V(gps_correction_count_), prior_velocity);
    estimated_state_.insert(B(gps_correction_count_), prior_imu_bias_);

    /* Assemble prior noise model and add it the graph */
    pose_noise_model_ = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
    velocity_noise_model_ = noiseModel::Isotropic::Sigma(3, 0.1);
    bias_noise_model_ = noiseModel::Isotropic::Sigma(6, 1e-3);

    /* Add to graph all prior factors (pose, velocity, bias) */
    graph_->add(PriorFactor<Pose3>(X(gps_correction_count_), prior_pose, pose_noise_model_));
    graph_->add(PriorFactor<Vector3>(V(gps_correction_count_), prior_velocity, velocity_noise_model_));
    graph_->add(PriorFactor<imuBias::ConstantBias>(B(gps_correction_count_), prior_imu_bias_, bias_noise_model_));

    /* Store previous state for the imu integration and the latest predicted outcome. */
    prev_state_ = NavState(prior_pose, prior_velocity);
    prop_state_ = prev_state_;
    prev_bias_ = prior_imu_bias_;

#ifdef USE_COMBINED
    imu_preintegrated_ = new PreintegratedCombinedMeasurements(preint_imu_params_, prior_imu_bias_);
#else
    imu_preintegrated_ = new PreintegratedImuMeasurements(preint_imu_params_, prior_imu_bias_);
#endif

}

void ImuFactors::SetImuPreintegrationParams()
{
    measured_acc_cov_ = Matrix33::Identity(3,3) * pow(accel_noise_sigma_, 2);
    measured_omega_cov_ = Matrix33::Identity(3,3) * pow(gyro_noise_sigma_, 2);
    integration_error_cov_ = Matrix33::Identity(3,3)*1e-8;                       // error committed in integrating position from velocities
    bias_acc_cov_ = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma_, 2);
    bias_omega_cov_ = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma_, 2);
    bias_acc_omega_int_ = Matrix::Identity(6,6)*1e-5;

    /* PreintegrationBase params */
    preint_imu_params_->accelerometerCovariance = measured_acc_cov_;          // acc white noise in continuous
    preint_imu_params_->integrationCovariance = integration_error_cov_;       // integration uncertainty continuous
    /* PreintegratedRotation params */
    preint_imu_params_->gyroscopeCovariance = measured_omega_cov_;            // gyro white noise in continuous
    /* PreintegrationCombinedMeasurements params */
    preint_imu_params_->biasAccCovariance = bias_acc_cov_;
    preint_imu_params_->biasOmegaCovariance = bias_omega_cov_;
    preint_imu_params_->biasAccOmegaInt = bias_acc_omega_int_;
}

void ImuFactors::UpdatePreintegration()
{
    /* Parse out first value */
    string value;
    getline(input_file_, value, ',');
    int type = atoi(value.c_str());

    if(type == 0){
        /* Update IMU measurement */
        ImuMeasurementUpdate();
    }
    else if(type == 1){
        /* GPS correction */
        GPSCorrection();

        /* reset integration and setting bias */
        imu_preintegrated_->resetIntegrationAndSetBias(prev_bias_);

        /* Calculate error and print result */
        PrintSimulationResult();
    }
    else{
        cerr << "ERROR parsing file\n";
    }
}

void ImuFactors::ImuMeasurementUpdate()
{
    imu_ = Eigen::Matrix<double,6,1>::Zero();
    string value;

    for(int i=0; i<5; i++){
        getline(input_file_, value, ',');
        imu_(i) = atof(value.c_str());
    }    
    getline(input_file_, value, '\n');
    imu_(5) = atof(value.c_str());

    /* Adding the IMU preintegration */
    imu_preintegrated_->integrateMeasurement(imu_.head<3>(), imu_.tail<3>(), dt_);
}

void ImuFactors::GPSCorrection()
{
    gps_ = Eigen::Matrix<double,7,1>::Zero();
    string value;

    for(int i=0; i<6; ++i){
        getline(input_file_, value, ',');
        gps_(i) = atof(value.c_str());
    }
    getline(input_file_, value, '\n');
    gps_(6) = atof(value.c_str());

    gps_correction_count_++;

#ifdef USE_COMBINED
    PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
    CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                    X(correction_count),   V(correction_count),
                                    B(correction_count-1), B(correction_count),
                                    *preint_imu_combined);
    graph->add(imu_factor);
#else 
    PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
    ImuFactor imu_factor(X(gps_correction_count_-1), V(gps_correction_count_-1),
                         X(gps_correction_count_),   V(gps_correction_count_),
                         B(gps_correction_count_-1),
                        *preint_imu);
    graph_->add(imu_factor);
    imuBias::ConstantBias zero_bias(Vector3(0,0,0), Vector3(0,0,0));
    graph_->add(BetweenFactor<imuBias::ConstantBias>(B(gps_correction_count_-1),
                                                     B(gps_correction_count_),
                                                     zero_bias, bias_noise_model_));
#endif

    noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,1,0);
    GPSFactor gps_factor(X(gps_correction_count_),
                        Point3(gps_(0),
                               gps_(1),
                               gps_(2)),
                        correction_noise);
    graph_->add(gps_factor);

    /* Optimize and compare results */
    prop_state_ = imu_preintegrated_->predict(prev_state_, prev_bias_);
    estimated_state_.insert(X(gps_correction_count_), prop_state_.pose());
    estimated_state_.insert(V(gps_correction_count_), prop_state_.v());
    estimated_state_.insert(B(gps_correction_count_), prev_bias_);

    LevenbergMarquardtOptimizer optimizer(*graph_, estimated_state_);
    Values result = optimizer.optimize();

    /* Overwrite the beginning of the preintegration for the next step. */
    prev_state_ = NavState(result.at<Pose3>(X(gps_correction_count_)),
                            result.at<Vector3>(V(gps_correction_count_)));
    prev_bias_ = result.at<imuBias::ConstantBias>(B(gps_correction_count_));
}

void ImuFactors::PrintSimulationResult()
{
     /* Print out the position and orientation error for comparison */
    Vector3 gtsam_position = prev_state_.pose().translation();
    Vector3 position_error_ = gtsam_position - gps_.head<3>();
    current_position_error_ = position_error_.norm();

    Quaternion gtsam_quat = prev_state_.pose().rotation().toQuaternion();
    Quaternion gps_quat(gps_(6), gps_(3), gps_(4), gps_(5));
    Quaternion quat_error = gtsam_quat * gps_quat.inverse();
    quat_error.normalize();
    Vector3 euler_angle_error(quat_error.x()*2,
                              quat_error.y()*2,
                              quat_error.z()*2);
    current_orientation_error_ = euler_angle_error.norm();

    /* display statistics */
    cout << "Position error: " << current_position_error_ << "\t" << "Angular error: " << current_orientation_error_ << "\n";
}