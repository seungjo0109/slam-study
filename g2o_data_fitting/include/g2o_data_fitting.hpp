// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

G2O_USE_OPTIMIZATION_LIBRARY(dense)

double CalculateError(int num_points, Eigen::Vector2d* points, const Eigen::Vector3d& circle)
{
    Eigen::Vector2d center = circle.head<2>();
    double radius = circle(2);
    double error = 0;
    for(int i=0; i<num_points; ++i){
        double d = (points[i] - center).norm() - radius;
        error += d*d;
    }
    return error;
}

/**
 * \brief a circle located at x,y with radius r
 */
class VertexCircle : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexCircle() {}

    virtual bool read(std::istream& /*is*/) { return false; }

    virtual bool write(std::ostream& /*os*/) const { return false; }

    virtual void setToOriginImpl() {
        std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    }

    virtual void oplusImpl(const double* update) {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * \brief measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle.
 * The error function computes the distance of the point to
 * the center minus the radius of the circle.
 */
class EdgePointOnCircle : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCircle() {}
    virtual bool read(std::istream& /*is*/) { return false; }
    virtual bool write(std::ostream& /*os*/) const { return false; }
    
    // void computeError()
    // {
    //     const VertexCircle* circle = static_cast<const VertexCircle*>(vertex(0));
    //     const Eigen::Vector2d& center = circle->estimate().head<2>();
    //     const double& radius = circle->estimate()(2);
    //     _error(0) = (measurement() - center).norm() - radius;
    // }
    template <typename T>
    bool operator()(const T* circle, T* error) const {
        typename g2o::VectorN<2, T>::ConstMapType center(circle);
        const T& radius = circle[2];
    
        error[0] = (measurement().cast<T>() - center).norm() - radius;
        return true;
    }
    
    G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff
};

class G2oDataFitting
{
    using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>;
    using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;

public:
    G2oDataFitting(int num_points, double radius, Eigen::Vector2d center)
        : num_points_(num_points), radius_(radius), center_(center)
    {
        /* Initialize Optimizer */
        optimizer_ = std::make_unique<g2o::SparseOptimizer>();
        optimizer_->setVerbose(false);
        /* Configure solver as Levenberg-Marquardt */
        solver_ = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolver>(g2o::make_unique<LinearSolver>()));
        solver_->setUserLambdaInit(0.5);
        optimizer_->setAlgorithm(solver_);

        // g2o::OptimizationAlgorithmProperty solver_property;   
        // optimizer_->setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense", solver_property));
    }
    ~G2oDataFitting(){
        delete(measurements_);
    }

    /* Generate noised circle sample data */
    void GenerateCircleSampleData()
    {  
        measurements_ = new Eigen::Vector2d[num_points_];
        g2o::Sampler::seedRand();

        for(int i=0; i<num_points_; ++i){
            double r = g2o::Sampler::gaussRand(radius_, 0.05);
            double angle = g2o::Sampler::uniformRand(0.0, 2.0 * M_PI);
            measurements_[i].x() = center_.x() + r*cos(angle);
            measurements_[i].y() = center_.y() + r*sin(angle);
        }
    }

    /* Set optimizer vertex & edges */
    void SetOptimizer()
    {
        vertex_circle_ = new VertexCircle();

        vertex_circle_->setId(0);
        vertex_circle_->setEstimate(
            Eigen::Vector3d(3.0, 3.0, 3.0));
        optimizer_->addVertex(vertex_circle_);

        for (int i=0; i<num_points_; ++i){
            EdgePointOnCircle*edge_circle_ = new EdgePointOnCircle();

            edge_circle_->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            edge_circle_->setVertex(0, vertex_circle_);
            edge_circle_->setMeasurement(measurements_[i]);
            optimizer_->addEdge(edge_circle_);
        }
    }

    /* Optimize function */
    void Optimize()
    {
        optimizer_->initializeOptimization();
        optimizer_->setVerbose(true);
        int iter = optimizer_->optimize(50);
        std::cout << "\noptimizer finished after " << iter << " iteration\n" << std::endl;
    }

    /* Print optimization result */
    void PrintResult()
    {
        std::cout << "Iterative least squares solution" << std::endl;
        std::cout << "center of the circle " << vertex_circle_->estimate().head<2>().transpose() << std::endl;
        std::cout << "radius of the cirlce " << vertex_circle_->estimate()(2) << std::endl;
        std::cout << "error " << CalculateError(num_points_, measurements_, vertex_circle_->estimate())     << std::endl;
        std::cout << std::endl;
    }

private:
    int num_points_;
    double radius_;
    Eigen::Vector2d center_;
    Eigen::Vector2d* measurements_;

    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    g2o::OptimizationAlgorithmLevenberg* solver_;
    VertexCircle* vertex_circle_;
};