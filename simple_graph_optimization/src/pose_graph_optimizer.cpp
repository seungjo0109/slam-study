#include "simple_graph_optimization/pose_graph_optimizer.hpp"

PoseGraphOptimizer::PoseGraphOptimizer()
{
    linear_solver_ = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    solver_ = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver_)));
    solver_->setUserLambdaInit(1);
    optimizer_ = new g2o::SparseOptimizer();
    optimizer_->setAlgorithm(solver_);
}

PoseGraphOptimizer::~PoseGraphOptimizer()
{
    delete(solver_);
    delete(optimizer_);
}