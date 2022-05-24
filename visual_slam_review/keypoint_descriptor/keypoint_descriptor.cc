#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main()
{
    // Load sample images
    cv::Mat img1 = cv::imread("../resource/1.png", IMREAD_COLOR);
    cv::Mat img2 = cv::imread("../resource/2.png", IMREAD_COLOR);
    if(img1.empty() || img2.empty()){
        cout << "Error: image load error" << endl;
    }

    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // Detect keypoint & Compute descriptor 
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);       

    // Compute computational time
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

    Mat out_img1;
    drawKeypoints(img1, keypoints1, out_img1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", out_img1);

    // Match descriptors
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors1.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
        }
    }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_match);
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);

    return 0;
}