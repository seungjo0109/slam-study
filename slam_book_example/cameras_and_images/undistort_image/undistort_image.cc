#include <iostream>

#include <opencv2/opencv.hpp>
std::string image_file = "./distorted.png";

int main(int argc, char** argv)
{
    // I this program we implement the undistortion by ourselves rather than using OpenCV

    // Radial-tangential model params
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // Intrinsics
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, 0);  // the image type is CV_8UC1
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1);   // the undistorted image

    // Computate the pixels in the undistorted one
    for(int v=0; v<rows; v++){
        for(int u=0; u<cols; u++){
            // Note we are computing the pixel of (u,v) in the undistorted image
            // according to the radial-tangential model, compute the coordinates
            // in the distorted image
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;

            // Check if the pixel is in the image boarder
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistorted.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistorted.at<uchar>(v, u) = 0;
            }
        }
    }

    cv::imshow("distorted_image", image);
    cv::imshow("undistorted_image", image_undistorted);
    cv::waitKey();

    return 0;
}