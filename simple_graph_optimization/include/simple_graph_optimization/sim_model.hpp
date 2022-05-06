/*

  The MIT License

  Copyright (c) 2020 Edward Im

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/
#pragma once

#include <vector>

#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "alias_tpyes.h"
#include "pose_graph_optimizer.hpp"

class SimModel
{
    using VecSE3Quat = std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>>;

public:
    explicit SimModel(int num_poses);
    ~SimModel();
    
    /* Generate ground truth */
    void GenerateTrueState();
    
    /* Add noise to true states */
    void GenerateSimState();
    
    /* Get true state */
    VecSE3Quat GetTrueState() {return true_poses_; }
    
    /* Get noise added simulation state */
    VecSE3Quat GetSimState() {return sim_poses_; }

    /* Get Optimizer */
    g2o::SparseOptimizer* GetOptimizer(){ return optimizer_->GetOptimizer(); }

    /* Get Optimization iteration count */
    int GetOptimizedIteration(){ return iter_optimization; }

    /* Add vertex data into optimizer */
    void AddVertex();

    /* Add Edge data into optimizer */
    void AddEdge();

    /* Optimize function */
    void Optimize(int iter) { 
        iter_optimization += iter;
        optimizer_->Optimize(iter); 
    }

    /* Optimize one time per each loop */
    void OptimizeOnce(){ 
        iter_optimization += 1;
        optimizer_->Optimize(1); 
    }

private:
    std::shared_ptr<PoseGraphOptimizer> optimizer_;

    int num_poses_{10};
    int iter_optimization{0};

    VecSE3Quat true_poses_;
    VecSE3Quat sim_poses_;

};

/* ~~~~~~~~~~~~~~~~ 
    SimVisualizer
 ~~~~~~~~~~~~~~~~ */ 
class SimVisualizer
{
public:
    /* Rviz visualization type */
    enum class DataType{
        TrueNode = 0,
        SimNode,
        OptimizedNode,
        TrueEdge,
        SimEdge,
        OptimizedEdge,
        SimText,
        IterText,
    };

    SimVisualizer();

    /* Set visualization_msgs::mgs::MarkerArray */
    void SetVisualizationMsg(DataType type, SimModel& sim_model);

    /* Return visualization_msgs::msgs::MarkerArray */
    visualization_msgs::msg::MarkerArray  GetVisualizationMsg(DataType type);

private:
    visualization_msgs::msg::MarkerArray true_node_;
    visualization_msgs::msg::MarkerArray true_edge_;
    visualization_msgs::msg::MarkerArray sim_node_;
    visualization_msgs::msg::MarkerArray sim_edge_;
    visualization_msgs::msg::MarkerArray optimized_node_;
    visualization_msgs::msg::MarkerArray optimized_edge_; 
    visualization_msgs::msg::MarkerArray sim_text_;
    visualization_msgs::msg::MarkerArray iter_text_;
};

/* ~~~~~~~~~~~~ 
    Sampling
 ~~~~~~~~~~~~ */ 
class Sampling
{
public:

    static int Uniform(int from, int to);
    
    static double Uniform();

    static double Gaussian(double sigma);
};