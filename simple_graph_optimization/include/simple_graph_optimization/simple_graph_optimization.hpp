#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "spdlog/spdlog.h"
#include "pose_graph_optimizer.hpp"
#include "sim_model.hpp"

class SimpleGraphOptimization : public rclcpp::Node
{
public:
    SimpleGraphOptimization() : Node("simple_graph_optimization"){
        
        /* Publisher & Subscriber initialization */
        InitRosPublisher();

        /* Node initialization */
        pose_graph_optimizer_   = std::make_unique<PoseGraphOptimizer>();
        sim_model_              = std::make_unique<SimModel>(15);
        visualizer_             = std::make_unique<SimVisualizer>();

        /* Simulation Initialization */ 
        sim_model_->GenerateTrueState();                            // Generte ground truth
        sim_model_->GenerateSimState();                             // Generate noised state
        sim_model_->AddVertex();                                    // Add vetex into optimizer
        sim_model_->AddEdge();                                      // Add edge into optimizer
        sim_model_->GetOptimizer()->initializeOptimization();       // Initialize optimizer
        sim_model_->GetOptimizer()->setVerbose(true);
        // sim_model_->Optimize(20);

        auto NodeRunner = [this]()-> void{
            /* Optimize graph once per loop iteration */
            sim_model_->OptimizeOnce();

            /* Set visualization messages */
            SetVisualizationMsg();
            
            /* Publish ROS messages */
            PublishMsg();
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds{100}, NodeRunner);  
    }

private:
    /* Publisher & Subscriber initialization */
    void InitRosPublisher(){
        true_node_pub_          = this->create_publisher<visualization_msgs::msg::MarkerArray>("true_node", 10);
        true_edge_pub_          = this->create_publisher<visualization_msgs::msg::MarkerArray>("true_edge", 10);
        sim_node_pub_           = this->create_publisher<visualization_msgs::msg::MarkerArray>("sim_node", 10);
        sim_edge_pub_           = this->create_publisher<visualization_msgs::msg::MarkerArray>("sim_edge", 10);
        optimized_node_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_node", 10);
        optimized_edge_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_edge", 10);
    }

    /* Set visualization messages */
    void SetVisualizationMsg(){
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::TrueNode, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::TrueEdge, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::SimNode, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::SimEdge, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::OptimizedNode, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::OptimizedEdge, *sim_model_);
    }

    /* Publish ROS messages */
    void PublishMsg(){
        true_node_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::TrueNode));
        true_edge_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::TrueEdge));
        sim_node_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::SimNode));
        sim_edge_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::SimEdge));
        optimized_node_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::OptimizedNode));
        optimized_edge_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::OptimizedEdge));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr true_node_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr true_edge_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sim_node_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sim_edge_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_node_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_edge_pub_;

    std::unique_ptr<PoseGraphOptimizer> pose_graph_optimizer_;
    std::unique_ptr<SimModel> sim_model_;
    std::unique_ptr<SimVisualizer> visualizer_;
};