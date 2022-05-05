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

#include <iostream>
#include <memory>
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

class PoseGraphOptimizer
{
public:
    PoseGraphOptimizer();
    ~PoseGraphOptimizer();

    /* Add vertex into optimizer */
    void AddVertex(g2o::VertexSE3Expmap* vtx){ optimizer_->addVertex(vtx); }
    
    /* Add edge into optimizer */
    void AddEdge(g2o::EdgeSE3Expmap* edge){ optimizer_->addEdge(edge); }
    
    // TODO: Review GetOptimizer function
    /* Return optimizer */
    g2o::SparseOptimizer* GetOptimizer(){ return optimizer_; }
    
    /* Optimize one time per each loop */
    void Optimize(int iter){ optimizer_->optimize(iter); }

private:

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver_;
    g2o::OptimizationAlgorithmLevenberg* solver_;
    g2o::SparseOptimizer* optimizer_;

};