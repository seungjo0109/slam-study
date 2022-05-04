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
        std::cout << "SimpleGraphOptimization" << std::endl;

        /* Publisher & Subscriber initialization */
        true_node_pub_          = this->create_publisher<visualization_msgs::msg::MarkerArray>("true_node", 10);
        sim_node_pub_           = this->create_publisher<visualization_msgs::msg::MarkerArray>("sim_node", 10);

        /* Node initialization */
        pose_graph_optimizer_   = std::make_unique<PoseGraphOptimizer>();
        sim_model_              = std::make_unique<SimModel>(15);
        visualizer_             = std::make_unique<SimVisualizer>();

        /* Run */ //TODO: move to NodeRunner
        sim_model_->GenerateTrueState();
        sim_model_->GenerateSimState();
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::TrueNode, *sim_model_);
        visualizer_->SetVisualizationMsg(SimVisualizer::DataType::SimNode, *sim_model_);
        // sim_model_->TestFunc();

        auto NodeRunner = [this]()-> void{
            true_node_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::TrueNode));
            sim_node_pub_->publish(visualizer_->GetVisualizationMsg(SimVisualizer::DataType::SimNode));
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds{1000}, NodeRunner);  
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr true_node_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sim_node_pub_;

    std::unique_ptr<PoseGraphOptimizer> pose_graph_optimizer_;
    std::unique_ptr<SimModel> sim_model_;
    std::unique_ptr<SimVisualizer> visualizer_;
};