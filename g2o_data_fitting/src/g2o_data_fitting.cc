#include "g2o_data_fitting.hpp"

int main()
{
    /* Simulation parameters */
    int num_points = 100;
    double radius = 5.0;
    Eigen::Vector2d center(0.0, 0.0);

    /* G2oDataFitting class instantiation */
    G2oDataFitting g2o_data_fitting(num_points, radius, center);

    /* Run */ 
    g2o_data_fitting.GenerateCircleSampleData();    // Generate circle measurement datas
    g2o_data_fitting.SetOptimizer();                // Set optimizer
    g2o_data_fitting.Optimize();                    // Optimize
    g2o_data_fitting.PrintResult();                 // Print optimize results

    return 0;
}