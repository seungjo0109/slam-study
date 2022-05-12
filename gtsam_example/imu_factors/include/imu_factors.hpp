#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X;  // Pose3    (x, y, r, p, y)
using symbol_shorthand::V;  // Vel      (xdot, ydot, zdot)    
using symbol_shorthand::B;  // Bias     (ax, ay, az, gx, gy, gz)

class ImuFactors
{
public:
    ImuFactors()
        // : data_file_name_ = 
    {
        
    }
    ~ImuFactors() = default;

private:

    std::string data_file_name_;

};