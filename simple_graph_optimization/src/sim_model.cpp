#include <rclcpp/rclcpp.hpp>

#include "simple_graph_optimization/sim_model.hpp"
#include "spdlog/spdlog.h"

SimModel::SimModel(int num_poses)
    : num_poses_(num_poses)
{

}

SimModel::~SimModel()
{

}

void SimModel::GenerateTrueState()
{
    for(int i=0; i<num_poses_; i++){
            Vec3 trans;
        if(i==0) {
        trans = Vec3(0,0,0);
        }
        else if(i>=1 && i <=7) {
        trans = Vec3(i + Sampling::Gaussian(.2),
                    Sampling::Gaussian(.2),
                    Sampling::Gaussian(.2));
        }
        else if(i==8) {
        trans = Vec3(8 + Sampling::Gaussian(.2),
                    7 - i + Sampling::Gaussian(.2),
                    Sampling::Gaussian(.2));
        }
        else if(i>=9) {
        trans = Vec3(17 - i + Sampling::Gaussian(.2),
                    -1 + Sampling::Gaussian(.2),
                    Sampling::Gaussian(.2));
        }

        Quaternion q;
        q.setIdentity(); // Set the initial rotation to identity.

        g2o::SE3Quat pose;
        pose.setRotation(q);
        pose.setTranslation(trans);

        true_poses_.push_back(pose);        
    }
}

void SimModel::GenerateSimState()
{
    for(int i=0; i<true_poses_.size(); i++){
        Vec3 trans(Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2),
                   Sampling::Gaussian(.2));
        Quaternion q;
        q.UnitRandom();

        g2o::SE3Quat pose;
        pose.setRotation(q);
        pose.setTranslation(true_poses_.at(i).translation() + trans);

        sim_poses_.push_back(pose);
    }
}

void SimModel::TestFunc()
{
    for(int i=0; i<true_poses_.size(); i++){

        auto true_node = visualization_msgs::msg::Marker();
        auto sim_node = visualization_msgs::msg::Marker();
        
        true_node.header.frame_id = "world";
        true_node.header.stamp = rclcpp::Clock().now();
        true_node.ns = "true_node";
        true_node.id = i;
        true_node.type = visualization_msgs::msg::Marker::SPHERE;
        true_node.scale.x = true_node.scale.y = true_node.scale.z = 0.2;
        true_node.pose.position.x = true_poses_.at(i).translation()[0];
        true_node.pose.position.y = true_poses_.at(i).translation()[1];
        true_node.pose.position.z = true_poses_.at(i).translation()[2];
        true_node.pose.orientation.x = true_poses_.at(i).rotation().x();
        true_node.pose.orientation.y = true_poses_.at(i).rotation().y();
        true_node.pose.orientation.z = true_poses_.at(i).rotation().z();
        true_node.pose.orientation.w = true_poses_.at(i).rotation().w();
        true_node.color.r = true_node.color.g = true_node.color.b = 0.0;
        true_node.color.a = 0.25;

        sim_node.header.frame_id = "world";
        sim_node.header.stamp = rclcpp::Clock().now();
        sim_node.ns = "sim_node";
        sim_node.id = i;
        sim_node.type = visualization_msgs::msg::Marker::SPHERE;
        sim_node.scale.x = sim_node.scale.y = sim_node.scale.z = 0.2;
        sim_node.pose.position.x = sim_poses_.at(i).translation()[0];
        sim_node.pose.position.y = sim_poses_.at(i).translation()[1];
        sim_node.pose.position.z = sim_poses_.at(i).translation()[2];
        sim_node.pose.orientation.x = sim_poses_.at(i).rotation().x();
        sim_node.pose.orientation.y = sim_poses_.at(i).rotation().y();
        sim_node.pose.orientation.z = sim_poses_.at(i).rotation().z();
        sim_node.pose.orientation.w = sim_poses_.at(i).rotation().w();
        sim_node.color.r = sim_node.color.g = sim_node.color.b = 0.0;
        sim_node.color.a = 1.0;

        true_node_.markers.push_back(true_node);
        sim_node_.markers.push_back(sim_node);
    }
}

int Sampling::Uniform(int from, int to)
{
    return static_cast<int>(UniformRand(from, to));
}
double Sampling::Uniform()
{
    return UniformRand(0.0, 1.0);
}
double Sampling::Gaussian(double sigma)
{
    return GaussRand(0.0, sigma);
}