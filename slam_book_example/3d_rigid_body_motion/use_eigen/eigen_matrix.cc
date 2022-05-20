#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{
    // All vectors and matrices in Eigen are Eigen::Matrix
    Eigen::Matrix<float, 2, 3> matrix_23;

    // Vector3d is essentially Eigen::Matrix<double, 3, 1>, which is a three-dimensional vector.
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float, 3, 1> vd_3d;

    // Matrix3d is essentially Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matirx_33 = Eigen::Matrix3d::Zero();
    
    // If you are not sure about the size of the matrix, you can use a matrix of 
    // dynamic size
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // simpler
    Eigen::MatrixXd matrix_x;


    return 0;
}