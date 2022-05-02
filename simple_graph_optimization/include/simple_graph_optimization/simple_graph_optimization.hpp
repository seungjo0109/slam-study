#pragma once

#include <iostream>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "pose_graph_optimizer.hpp"

class SimpleGraphOptimization : public rclcpp::Node
{
public:
    SimpleGraphOptimization(rclcpp::Node::SharedPtr nh) : Node("simple_graph_optimization"), nh_(nh){

        std::cout << "SimpleGraphOptimization" << std::endl;

        auto NodeRunner = [this]()-> void{
            std::cout<< "node runner" << std::endl;
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds{500}, NodeRunner);
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::SharedPtr nh_;
};