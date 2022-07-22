#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <pangolin/pangolin.h>

#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/se3.hpp>

using TrajectoryType = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;
using Vector6d = Eigen::Matrix<double,6,1>;

void ShowPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>>& point_cloud);

int main()
{
    std::vector<cv::Mat> color_imgs, depth_imgs;
    TrajectoryType poses;       // camera poses

    std::ifstream fin("./pose.txt");
    if(!fin){
        std::cerr << "Please run the program in the directory that has pose.txt" << std::endl;
        return 0;
    }

    for(int i=0; i<5; i++){
        boost::format fmt("./%s/%d.%s");    // the image format
        color_imgs.push_back(cv::imread((fmt % "color" % (i+1)%"png").str()));
        depth_imgs.push_back(cv::imread((fmt % "depth" % (i+1)%"pgm").str(), -1));  // use -1 flag to load the depth image

        double data[7] = {0, };
        for(auto &d : data){
            fin >> d;
        }
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]), Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose); 
    }

    // Compute the point cloud using camera instrinsics
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depth_scale = 1000.0;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> point_cloud;
    point_cloud.reserve(1000000);

    for (int i = 0; i < 5; i++) {
        std::cout << "Converting RGBD images: " << i + 1 << std::endl;
        cv::Mat color = color_imgs[i];
        cv::Mat depth = depth_imgs[i];
        Sophus::SE3d T = poses[i];

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u];   // depth value is 16-bit
                if (d == 0){
                    continue;   // 0 means no valid value 
                }

                Eigen::Vector3d point;
                point[2] = double(d) / depth_scale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d point_world = T * point;

                Vector6d p;
                p.head<3>() = point_world;
                p[5] = color.data[v * color.step + u * color.channels()];       // blue
                p[4] = color.data[v * color.step + u * color.channels() + 1];   // green
                p[3] = color.data[v * color.step + u * color.channels() + 2];   // red
                point_cloud.push_back(p);
            }
    }

    std::cout << "global point cloud has " << point_cloud.size() << "points." << std::endl;
    ShowPointCloud(point_cloud);
    return 0;
}

void ShowPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>>& point_cloud) {

    if (point_cloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: point_cloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}