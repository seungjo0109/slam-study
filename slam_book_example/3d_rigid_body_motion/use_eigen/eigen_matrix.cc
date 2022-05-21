#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main()
{
    // All vectors and matrices in Eigen are Eigen::Matrix
    Eigen::Matrix<float, 2, 3> matrix_23;

    // Vector3d is essentially Eigen::Matrix<double, 3, 1>, which is a three-dimensional vector.
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float, 3, 1> vd_3d;

    // Matrix3d is essentially Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    
    // If you are not sure about the size of the matrix, you can use a matrix of 
    // dynamic size
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // simpler
    Eigen::MatrixXd matrix_x;

    // input data (initialization)
    matrix_23 << 1, 2, 3, 4, 5, 6;
    std::cout << "matrix 2x3 from 1 to 6" << matrix_23 << std::endl;

    // Use () to access elements in the matrix
    std::cout << "print matrix 2x3 " << std::endl;
    for(int i=0; i<2; i++){
        for(int j=0; j<3; j++){
            std::cout << matrix_23(i,j) << "\t";
        }
        std::cout << std::endl;
    }

    // We can easily multiply a matrix with a vector (but actually still matrices and matrices)
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    // In Eigen you can't mix two different types of matrices
    // It should be explicitly converted
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1,2,3,4,5,6]*[3,2,1]=" << result.transpose() << std::endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    std::cout << "[1,2,3,4,5,6]*[4,5,6]: "<< result2.transpose() << std::endl;

    // Some matrix operations
    matrix_33 = Eigen::Matrix3d::Random();
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "times 10: \n" << 10* matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;

    // Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values= \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors= \n" << eigen_solver.eigenvectors() << std::endl;
 
    // Solving equations
    // We solve the equation of matrix_NN ∗ x = v_Nd

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN =
        Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();      // Guarantee semi-positive definite
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();     // timing
    // Direct inversion
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is " << 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << std::endl;

    // Usually solved by matrix decomposition, such as QR decomposition, the speed
    // will be much faster
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "time of Qr decomposition is " << 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << std::endl;

    // For positive definite matrices, you can also use cholesky decomposition to
    // solve equations.    
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "time of ldlt decomposition is " << 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    return 0;
}