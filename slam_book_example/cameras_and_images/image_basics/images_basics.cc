#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    cv::Mat image;
    image = cv::imread(argv[1]);    // call cv::imread to read the image from file

    // Check the data is correctly loaded
    if(image.data == nullptr){
        std::cerr << "file" << argv[1] << " not exist." << std::endl;
        return 0;
    }

    // Print some basic information
    std::cout << "Image colums: " << image.cols << ", rows: " << image.rows
        << ", channels: " << image.channels() << std::endl;
    // Use cv::imshow to show the image
    cv::imshow("image", image);
    // Display and wait for the keyboard input 
    cv::waitKey(0);     

    // Check image type
    if(image.type() != CV_8UC1 && image.type() != CV_8UC3){
        // We need to grayscale image or RGB image
        std::cout << "image type is incorrect." << std::endl;
        return 0;
    }

    // Check hte pixels
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for(size_t y=0; y<image.rows; y++){
        // Use cv::Mat::ptr to get the pointer of each row
        unsigned char *row_ptr = image.ptr<unsigned char>(y); // row prt is the pointer to y_th row

        for(size_t x=0; x<image.cols; x++){
            // Read the pixel on (x,y), x=column, y=row
            unsigned char* data_ptr = &row_ptr[x*image.channels()];     // data_ptr is the pointer to (x,y)
            // Visit the pixel in each channel
            for(int c=0; c!= image.channels(); c++){
                unsigned char data = data_ptr[c];   // data should be pixel of I(x,y) in c-th channel
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);

    std::cout << "time used: " << time_used.count() << " seconds." << std::endl;

    // Copying cv::Mat
    // operator = will not copy the image data, but only the reference
    cv::Mat image_another = image;
    // Changing image_another will also change image
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);   // set top-left 100*100 block to zero
    // Use cv::imshow to show the image
    cv::imshow("image_another", image_another);
    // Display and wait for the keyboard input 
    cv::waitKey(0);  

    // Use cv::Mat::clone to actually clone the data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    // Use cv::imshow to show the image
    cv::imshow("image_clone", image_clone);
    // Display and wait for the keyboard input 
    cv::waitKey(0);  

    cv::destroyAllWindows();

    return 0;
}