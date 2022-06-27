#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/se3.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main()
{
    // Rotation matrix with 90 degrees along Z axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Eigen::Quaterniond q(R);

    // Sophus::SO3d can be constructed from rotation matrix
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    std::cout << "SO(3) from matrix: \n" << SO3_R.matrix() << std::endl << std::endl;
    std::cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << std::endl << std::endl;
    std::cout << "They are equal \n" << std::endl << std::endl;

    // Use logarithmic map to get the Lie algebra
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3 = " << so3.transpose() << std::endl << std::endl;

    // hat is from vector to skew-symmetric matrix
    std::cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << std::endl << std::endl;
    
    // inversely from matrix to vector
    std::cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl << std::endl;

    // Update by perturbation model
    Eigen::Vector3d update_so3(1e-4,0,0);   // this is a small update
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    std::cout << "SO3 updated = \n" << SO3_updated.matrix() << std::endl << std::endl;

    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl << std::endl;
    // Similar for SE(3)
    Eigen::Vector3d t(1,0,0);               // translation 1 along X
    Sophus::SE3d SE3_Rt(R,t);               // construction SE3 from R,t
    Sophus::SE3d SE3_qt(q,t);               // or q,t
    std::cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << std::endl << std::endl;
    std::cout << "SE3 from q,t= \n" << SE3_qt.matrix() << std::endl << std::endl;

    // Lie Algebra
    Vector6d se3 = SE3_Rt.log();
    std::cout << "se3 = " << se3.transpose() << std::endl << std::endl;

    // The output shows Sophus puts the translation at first in se(3), then rotation
    std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << std::endl << std::endl;
    std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl << std::endl;

    // Finally the update
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;

    return 0;
}