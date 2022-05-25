#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void FindFeatureMathes(
    const Mat &img1, const Mat &img2,
    vector<KeyPoint> &keypoints1,
    vector<KeyPoint> &keypoints2,
    vector<DMatch> &matches);

void PoseEstimation(
  vector<KeyPoint> keypoints1,
  vector<KeyPoint> keypoints2,
  vector<DMatch> matches,
  Mat &R, Mat &t);

Point2d Pixel2Cam(const Point2d &p, const Mat &K);

int main()
{
    Mat img1 = imread("../resource/1.png",IMREAD_COLOR);
    Mat img2 = imread("../resource/2.png",IMREAD_COLOR);
    
    vector<KeyPoint> keypoints1, keypoints2;
    vector<DMatch> matches;
    FindFeatureMathes(img1, img2, keypoints1, keypoints2, matches);

    Mat R, t;
    PoseEstimation(keypoints1, keypoints2, matches, R, t);

    Mat t_x =
    (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);

    cout << "t^R=" << endl << t_x * R << endl;

    return 0;
}

void FindFeatureMathes(
    const Mat &img1, const Mat &img2,
    vector<KeyPoint> &keypoints1,
    vector<KeyPoint> &keypoints2,
    vector<DMatch> &matches)
{
    Mat descriptors1, descriptors2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // Detect keypoint & Compute descriptor 
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);
    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);    

    vector<DMatch> match;
    matcher->match(descriptors1, descriptors2, match);

    double min_dist = 10000, max_dist = 0;

    for (int i = 0; i < descriptors1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    for (int i = 0; i < descriptors1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
        matches.push_back(match[i]);
        }
    }
}

void PoseEstimation(
    vector<KeyPoint> keypoints1,
    vector<KeyPoint> keypoints2,
    vector<DMatch> matches,
    Mat &R, Mat &t)
{
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<Point2f> points1;
    vector<Point2f> points2;

    for(int i=0; i<(int)matches.size(); i++){
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }

    // Calculate fundamental matrix
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, FM_8POINT);

    Point2d principal_point(325.1, 249.7);  
    double focal_length = 521;      

    // Calculate essential matrix
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

Point2d Pixel2Cam(const Point2d &p, const Mat &K)
{
    return Point2d
        (
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
}

