/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

/* Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor */
// #define USE_COMBINED

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;  // Pose3    (x, y, r, p, y)
using symbol_shorthand::V;  // Vel      (xdot, ydot, zdot)    
using symbol_shorthand::B;  // Bias     (ax, ay, az, gx, gy, gz)

class ImuFactors
{
public:
    ImuFactors();
    ~ImuFactors();

    /* Set IMU and GPS data from CSV */
    void SetInputFile(string input_data) { input_file_ = ifstream(input_data.c_str()); }

    /* Initialize imu preintegration */
    void InitImuPreintegration();

    /* Check validation of the file */
    bool FileIsValid() { return input_file_.good(); }

    /* Update preintegration */
    void UpdatePreintegration();

    /* Print Simulation result */
    void PrintSimulationResult();

private:
    /* use the sensor specs to build the noise model for the IMU factor */
    void SetImuPreintegrationParams();

    /* IMU measurement update function */
    void ImuMeasurementUpdate();

    /* GPS correction with GPS measurement */
    void GPSCorrection();


private:
    /*
         we read IMU and GPS data from a CSV file, with the following format:
        A row starting with "i" is the first initial position formatted with
        N, E, D, qx, qY, qZ, qW, velN, velE, velD
        A row starting with "0" is an imu measurement
        (body frame - Forward, Right, Down)
        linAccX, linAccY, linAccZ, angVelX, angVelY, angVelX
        A row starting with "1" is a gps correction formatted with
        N, E, D, qX, qY, qZ, qW
        Note that for GPS correction, we're only using the position not the
        rotation. The rotation is provided in the file for ground truth comparison.
    */
    ifstream input_file_;

    /* 
        This will either be PreintegratedImuMeasurements (for ImuFactor) or
        PreintegratedCombinedMeasurements (for CombinedImuFactor)            
    */
    PreintegrationType* imu_preintegrated_;
    NonlinearFactorGraph* graph_;
    Values estimated_state_;
    NavState prev_state_;
    NavState prop_state_;
    imuBias::ConstantBias prev_bias_;
    Eigen::Matrix<double,6,1> imu_;
    Eigen::Matrix<double,7,1> gps_;

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> preint_imu_params_;    

    /* simulation params */
    const double dt_{0.005};
    int gps_correction_count_{0};
    double current_position_error_;
    double current_orientation_error_;

    /* imu sensor specs */
    const double accel_noise_sigma_{0.0003924};
    const double gyro_noise_sigma_{0.000205689024915};
    const double accel_bias_rw_sigma_{0.004905};
    const double gyro_bias_rw_sigma_{0.000001454441043};
    imuBias::ConstantBias prior_imu_bias_;                  // assume zero initial bias

    noiseModel::Diagonal::shared_ptr pose_noise_model_;
    noiseModel::Diagonal::shared_ptr velocity_noise_model_;
    noiseModel::Diagonal::shared_ptr bias_noise_model_;

    Matrix33 measured_acc_cov_;
    Matrix33 measured_omega_cov_;
    Matrix33 integration_error_cov_;
    Matrix33 bias_acc_cov_;
    Matrix33 bias_omega_cov_;
    Matrix66 bias_acc_omega_int_;
};