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

        true_node_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("true_node", 10);
        sim_node_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sim_node", 10);

        pose_graph_optimizer_ = std::make_unique<PoseGraphOptimizer>();
        sim_model_ = std::make_unique<SimModel>(15);

        sim_model_->GenerateTrueState();
        sim_model_->GenerateSimState();
        sim_model_->TestFunc();

        auto NodeRunner = [this]()-> void{
            true_node_pub_->publish(sim_model_->true_node_);
            sim_node_pub_->publish(sim_model_->sim_node_);
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds{1000}, NodeRunner);  
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr true_node_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sim_node_pub_;

    std::unique_ptr<PoseGraphOptimizer> pose_graph_optimizer_;
    std::unique_ptr<SimModel> sim_model_;
};