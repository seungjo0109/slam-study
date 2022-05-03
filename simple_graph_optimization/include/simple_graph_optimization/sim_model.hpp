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

class Sampling
{
public:
    static int Uniform(int from, int to);
    
    static double Uniform();

    static double Gaussian(double sigma);
};

class SimModel
{
public:
    explicit SimModel(int num_poses);
    ~SimModel();
    
    /* Generate ground truth */
    void GenerateTrueState();
    /* Add noise to true states */
    void GenerateSimState();

    void TestFunc();

    visualization_msgs::msg::MarkerArray true_node_;    //FIXME
    visualization_msgs::msg::MarkerArray sim_node_;     // FIXME

private:

    int num_poses_{10};
    int vertex_id_{0};

    std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> true_poses_;
    std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> sim_poses_;
};