#include <rclcpp/rclcpp.hpp>

#include "simple_graph_optimization/sim_model.hpp"
#include "spdlog/spdlog.h"

SimModel::SimModel(int num_poses)
    : num_poses_(num_poses)
{
    optimizer_ = std::make_unique<PoseGraphOptimizer>();
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

void SimModel::AddVertex()
{
    for(int i=0; i<sim_poses_.size(); i++){
        g2o::VertexSE3Expmap *vtx = new g2o::VertexSE3Expmap();
        
        if(i==0){
            vtx->setFixed(true);
        }
        vtx->setId(i);
        vtx->setEstimate(sim_poses_.at(i));
        optimizer_->AddVertex(vtx);
    }
}

void SimModel::AddEdge()
{
    for(int i=1; i<true_poses_.size(); i++){
        g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
        g2o::SE3Quat relative_pos = true_poses_.at(i-1).inverse() * true_poses_.at(i);
        
        edge->setMeasurement(relative_pos);

        MatXX A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
        MatXX information = A.transpose() * A;

        edge->setInformation(information);
        edge->vertices()[0] = optimizer_->GetOptimizer()->vertex(i-1);
        edge->vertices()[1] = optimizer_->GetOptimizer()->vertex(i);

        optimizer_->AddEdge(edge);
    }

    // Add non-temporal edges. (5 & 11)
    g2o::EdgeSE3Expmap* e511(new g2o::EdgeSE3Expmap());
    g2o::SE3Quat relative_pos = true_poses_.at(5).inverse() * true_poses_.at(11);
    e511->setMeasurement(relative_pos);
    MatXX A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
    MatXX information = A.transpose() * A;
    e511->setInformation(information);
    e511->vertices()[0] = optimizer_->GetOptimizer()->vertex(5);
    e511->vertices()[1] = optimizer_->GetOptimizer()->vertex(11);
    optimizer_->AddEdge(e511);

    // Add non-temporal edges. (3 & 14)
    g2o::EdgeSE3Expmap* e314(new g2o::EdgeSE3Expmap());
    relative_pos = true_poses_.at(3).inverse() * true_poses_.at(14);
    e511->setMeasurement(relative_pos);
    A = Eigen::MatrixXd::Random(6,6).cwiseAbs();
    information = A.transpose() * A;
    e511->setInformation(information);
    e511->vertices()[0] = optimizer_->GetOptimizer()->vertex(3);
    e511->vertices()[1] = optimizer_->GetOptimizer()->vertex(14);
    optimizer_->AddEdge(e314);
}

void SimModel::TestFunc()
{

}

/* ~~~~~~~~~~~~~~~~ 
    SimVisualizer
 ~~~~~~~~~~~~~~~~ */

SimVisualizer::SimVisualizer()
{
    true_node_.markers.clear();
    true_edge_.markers.clear();
    sim_node_.markers.clear();
    optimized_node_.markers.clear();
    optimized_edge_.markers.clear();
}

void SimVisualizer::SetVisualizationMsg(DataType type, SimModel& sim_model)
{
    switch (type)
    {
        case DataType::TrueNode:
        {   
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
        }
        case DataType::SimNode:
        {
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
                sim_node.color.r = 1.0;
                sim_node.color.g = sim_node.color.b = 0.0;
                sim_node.color.a = 0.25;

                sim_node_.markers.push_back(sim_node);
            }
            spdlog::info("true poses size: {}", sim_model.GetSimState().size());
            break;
        }
        case DataType::OptimizedNode:
        {
            visualization_msgs::msg::Marker node;
            double pos_init[3];

            for(int i = 0; i<sim_model.GetSimState().size(); i++){
                g2o::VertexSE3Expmap* vertex = static_cast<g2o::VertexSE3Expmap*>(sim_model.GetOptimizer()->vertex(i));
                Isometry optimized_pos = vertex->estimate();
                Quaternion q = (Quaternion)optimized_pos.rotation();

                node.header.frame_id = "world";
                node.header.stamp = rclcpp::Clock().now();
                node.ns = "optimized_node";
                node.id = i;
                node.type = visualization_msgs::msg::Marker::SPHERE;
                node.scale.x = node.scale.y = node.scale.z = 0.1;
                node.color.r = node.color.g = 0.0;
                node.color.b = 1.0;
                node.color.a = 1.0;

                if(i == 0){
                    pos_init[0] = optimized_pos.translation()[0];
                    pos_init[1] = optimized_pos.translation()[1];
                    pos_init[2] = optimized_pos.translation()[2];
                }

                node.pose.position.x = optimized_pos.translation()[0] - pos_init[0];
                node.pose.position.y = optimized_pos.translation()[1] - pos_init[1];
                node.pose.position.z = optimized_pos.translation()[2] - pos_init[2];

                node.pose.orientation.x = q.x();
                node.pose.orientation.y = q.y();
                node.pose.orientation.z = q.z();
                node.pose.orientation.w = q.w();

                if(node.pose.orientation.w < 0){
                    node.pose.orientation.x *= -1;
                    node.pose.orientation.y *= -1;
                    node.pose.orientation.z *= -1;
                    node.pose.orientation.w *= -1;
                }

                optimized_node_.markers.push_back(node);      
            }
            break;   
        }
        case DataType::TrueEdge:
        {
            geometry_msgs::msg::Point p1;
            geometry_msgs::msg::Point p2;

            for(int i=1; i<sim_model.GetTrueState().size(); i++){

                visualization_msgs::msg::Marker edge;

                edge.header.frame_id = "world";
                edge.header.stamp = rclcpp::Clock().now();
                edge.ns = "true_edge";
                edge.id = i;
                edge.type = visualization_msgs::msg::Marker::LINE_LIST;
                edge.scale.x = 0.05;
                edge.pose.orientation.w = 1.0;
                edge.color.r = edge.color.g = edge.color.b = 0.0;
                edge.color.a = 0.1;           

                p1.x = sim_model.GetTrueState()[i-1].translation()[0];
                p1.y = sim_model.GetTrueState()[i-1].translation()[1];
                p1.z = sim_model.GetTrueState()[i-1].translation()[2];
                edge.points.push_back(p1);

                p2.x = sim_model.GetTrueState()[i].translation()[0];
                p2.y = sim_model.GetTrueState()[i].translation()[1];
                p2.z = sim_model.GetTrueState()[i].translation()[2];
                edge.points.push_back(p2);

                true_edge_.markers.push_back(edge);
            }
            break;
        }
        case DataType::OptimizedEdge:
        {
            double pos_init[3];
            geometry_msgs::msg::Point p1;
            geometry_msgs::msg::Point p2;

            for(int i=1; i<sim_model.GetSimState().size(); i++){

                visualization_msgs::msg::Marker edge;
                g2o::VertexSE3Expmap* start_vertex = static_cast<g2o::VertexSE3Expmap*>(sim_model.GetOptimizer()->vertex(i-1));
                g2o::VertexSE3Expmap* end_vertex = static_cast<g2o::VertexSE3Expmap*>(sim_model.GetOptimizer()->vertex(i));

                Isometry start_optimized_pos = start_vertex->estimate();
                Isometry end_optimized_pos = end_vertex->estimate();

                edge.header.frame_id = "world";
                edge.header.stamp = rclcpp::Clock().now();
                edge.ns = "optimized_edge";
                edge.id = i;
                edge.type = visualization_msgs::msg::Marker::LINE_LIST;
                edge.scale.x = 0.03;
                edge.pose.orientation.w = 1.0;
                edge.color.r = edge.color.g = 0.0;
                edge.color.b = 1.0;
                edge.color.a = 0.25;    

                if(i == 1){
                    pos_init[0] = start_optimized_pos.translation()[0];
                    pos_init[1] = start_optimized_pos.translation()[1];
                    pos_init[2] = start_optimized_pos.translation()[2];
                }

                p1.x = start_optimized_pos.translation()[0] - pos_init[0];
                p1.y = start_optimized_pos.translation()[1] - pos_init[1];
                p1.z = start_optimized_pos.translation()[2] - pos_init[2];
                edge.points.push_back(p1);

                p2.x = end_optimized_pos.translation()[0] - pos_init[0];
                p2.y = end_optimized_pos.translation()[1] - pos_init[1];
                p2.z = end_optimized_pos.translation()[2] - pos_init[2];
                edge.points.push_back(p2);

                optimized_edge_.markers.push_back(edge);
            }  
            break;   
        }
    }
}

visualization_msgs::msg::MarkerArray SimVisualizer::GetVisualizationMsg(DataType type)
{
    switch(type)
    {
        case DataType::TrueNode:
            return true_node_;
            break;
        case DataType::TrueEdge:
            return true_edge_;
            break;
        case DataType::SimNode:
            return sim_node_;
            break;
        case DataType::OptimizedNode:
            return optimized_node_;
            break;
        case DataType::OptimizedEdge:
            return optimized_edge_;
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