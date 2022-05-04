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

}

/* ~~~~~~~~~~~~~~~~ 
    SimVisualizer
 ~~~~~~~~~~~~~~~~ */ 
void SimVisualizer::SetVisualizationMsg(DataType type, SimModel& sim_model)
{
    switch (type)
    {
    case DataType::TrueNode:
        for(auto i=0; i<sim_model.GetTrueState().size(); i++){
            auto true_node = visualization_msgs::msg::Marker();
            
            true_node.header.frame_id = "world";
            true_node.header.stamp = rclcpp::Clock().now();
            true_node.ns = "true_node";
            true_node.id = i;
            true_node.type = visualization_msgs::msg::Marker::SPHERE;
            true_node.scale.x = true_node.scale.y = true_node.scale.z = 0.2;
            true_node.pose.position.x = sim_model.GetTrueState().at(i).translation()[0];
            true_node.pose.position.y = sim_model.GetTrueState().at(i).translation()[1];
            true_node.pose.position.z = sim_model.GetTrueState().at(i).translation()[2];
            true_node.pose.orientation.y = sim_model.GetTrueState().at(i).rotation().y();
            true_node.pose.orientation.z = sim_model.GetTrueState().at(i).rotation().z();
            true_node.pose.orientation.w = sim_model.GetTrueState().at(i).rotation().w();
            true_node.pose.orientation.x = sim_model.GetTrueState().at(i).rotation().x();
            true_node.color.r = true_node.color.g = true_node.color.b = 0.0;
            true_node.color.a = 0.25;

            true_node_.markers.push_back(true_node);
        }
        break;
    case DataType::SimNode:
        for(auto i=0; i<sim_model.GetSimState().size(); i++){
            auto sim_node = visualization_msgs::msg::Marker();
            
            sim_node.header.frame_id = "world";
            sim_node.header.stamp = rclcpp::Clock().now();
            sim_node.ns = "sim_node";
            sim_node.id = i;
            sim_node.type = visualization_msgs::msg::Marker::SPHERE;
            sim_node.scale.x = sim_node.scale.y = sim_node.scale.z = 0.2;
            sim_node.pose.position.x = sim_model.GetSimState().at(i).translation()[0];
            sim_node.pose.position.y = sim_model.GetSimState().at(i).translation()[1];
            sim_node.pose.position.z = sim_model.GetSimState().at(i).translation()[2];
            sim_node.pose.orientation.y = sim_model.GetSimState().at(i).rotation().y();
            sim_node.pose.orientation.z = sim_model.GetSimState().at(i).rotation().z();
            sim_node.pose.orientation.w = sim_model.GetSimState().at(i).rotation().w();
            sim_node.pose.orientation.x = sim_model.GetSimState().at(i).rotation().x();
            sim_node.color.r = sim_node.color.g = sim_node.color.b = 0.0;
            sim_node.color.a = 0.25;

            sim_node_.markers.push_back(sim_node);
        }
        spdlog::info("true poses size: {}", sim_model.GetSimState().size());
        break;

    default:

        break;
    }
}

visualization_msgs::msg::MarkerArray SimVisualizer::GetVisualizationMsg(DataType type)
{
    switch(type)
    {
        case DataType::TrueNode:
            return true_node_;
            break;
        case DataType::SimNode:
            return sim_node_;
            break;
    }
}

/* ~~~~~~~~~~~~ 
    Sampling
 ~~~~~~~~~~~~ */ 
static double UniformRand(double lower_bound, double upper_bound)
{
    return lower_bound + ((double) std::rand()/(RAND_MAX + 1.0)) * (upper_bound - lower_bound);
}
static double GaussRand(double mean, double sigma)
{
    double x, y, r2;
    do {
        x = -1.0 + 2.0 * UniformRand(0.0, 1.0);
        y = -1.0 + 2.0 * UniformRand(0.0, 1.0);
        r2 = x*x + y*y;
    } while(r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
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