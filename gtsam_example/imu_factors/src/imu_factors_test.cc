#include <chrono>
#include <thread>

#include "imu_factors.hpp"

int main(int argc, char* argv[])
{
    ImuFactors imu_factors;

    try {
        imu_factors.SetInputFile(argv[1]);
    } catch (...){
        cout << "ERROR: Input file error" << endl;
        return 1;
    }

    /* Initialize imu preintegration */
    imu_factors.InitImuPreintegration();

    while(imu_factors.FileIsValid()){
        imu_factors.UpdatePreintegration();
    }

    return 0;
}