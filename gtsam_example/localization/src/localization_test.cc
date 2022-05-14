#include "localization.hpp"

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise);

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  auto unaryNoise =
      noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}