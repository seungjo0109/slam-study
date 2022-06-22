#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main()
{
    // 3D rotation matrix directly using Matrix3d or Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    /*
        The rotation vector uses AngleAxis, the underlying layer is not directly Matrix,
        but the operation can be treated as a matrix (because the operator is overloaded)
    */
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0, 0, 1));        // Rotate 45 degrees along the Z axis

    std::cout.precision(3);
    std::cout << "ratation matrix = \n" << rotation_vector.matrix() << std::endl << std::endl;     // Convert to matrix with matrix

    // Can also be assigned directly
    rotation_matrix = rotation_vector.toRotationMatrix();

    // Coordinate transformation with AngleAxis
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    std::cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << std::endl << std::endl;

    // Or use a rotation matrix
    v_rotated = rotation_matrix * v;
    std::cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << std::endl << std::endl;

    // Euler angle: You can convert the rotation matrix directly into Euler angles
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);      // ZYX order
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl << std::endl;

    // Euclidean transformation matrix using Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();        // Although called 3d, it is essentially a 4*4 matrix
    T.rotate(rotation_vector);                                  // Rotate according to rotation_vector
    T.pretranslate(Eigen::Vector3d(1,3,4));                     // Set the translation vector to (1,3,4)
    std::cout << "Transformation matrix = \n" << T.matrix() <<  std::endl << std::endl;

    // Use the transformation matrix for coordinate transformation
    Eigen::Vector3d v_transformed = T * v;  // Equivalent to R*v + t
    std::cout << "v_transformed = " << v_transformed.transpose() << std::endl << std::endl;

    // For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d.

    // Quaternion
    // You can assign AngleAxis directly to quaternions. and vice versa
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);         // Note that the order of coeffs is (x, y, z, w), w is the real part.
    std::cout << "quaternion from rotation vector = " << q.coeffs().transpose() << std::endl << std::endl;

    // Can also assign a rotation matrix to it
    q = Eigen::Quaterniond(rotation_matrix);
    std::cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << std::endl << std::endl;

    // Rotate a vector with a quaternion and use overloaded multiplication
    v_rotated = q*v;        // Note that the math is qvq^{-1}
    std::cout << "(1,0,0) after rotation = " << v_rotated.transpose() << std::endl;

    // Expressed by regular vector multiplication, it should be calculated as follows
    std::cout << "should be equal to " << (q * Eigen::Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << std::endl;

    return 0;
}