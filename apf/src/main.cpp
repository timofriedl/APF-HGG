#include <iostream>
#include <Eigen/Dense>
#include "gateway.h"

int main() {
    std::array<double, ACT_COUNT> q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.5};
    std::array<double, 3> targetPos = {1, 2, 3};
    double targetGripper = 1.0;
    std::vector<double> obstacleAttributes = {
            0, 1, 2, 3, 4, 5, 1.0,
            1, 2, 3, 4, 5, 6, 2.0,
            2, 3, 4, 5, 6, 7, 3.0,
            3, 4, 5, 6, 7, 8, 4.0,
            4, 5, 6, 7, 8, 9, 5.0,
            5, 6, 7, 8, 9, 10, 6.0
    };
    double dt = 0.001;

    std::array<double, ACT_COUNT> integralValues{};
    std::array<double, ACT_COUNT> prevErrorValues{};
    std::array<double, ACT_COUNT> forceResult{};

    auto start = std::chrono::high_resolution_clock::now();

    step(q.data(), targetPos.data(), targetGripper, obstacleAttributes.data(), obstacleAttributes.size() / 7,
         dt, integralValues.data(), prevErrorValues.data(), forceResult.data());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Elapsed time: " << duration.count() * 1000 << " ms (" << (1.0 / duration.count()) << " Hz) \n";
}