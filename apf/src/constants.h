#ifndef APF_HGG_CORE_CONSTANTS_H
#define APF_HGG_CORE_CONSTANTS_H

#include <cstddef>
#include <Eigen/Dense>

// Robot
constexpr size_t JOINT_COUNT = 7;

// Jacobian
constexpr double DERIVATIVE_EPSILON = 1E-5;

// PID Controller
constexpr double K_p = 2000.0;
constexpr double K_i = 400.0;
constexpr double K_d = 50.0;

constexpr std::array<double, JOINT_COUNT> PID_MIN_FORCE{-87.0, -87.0, -87.0, -87.0, -12.0, -12.0, -12.0};
constexpr std::array<double, JOINT_COUNT> PID_MAX_FORCE{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

// APF
constexpr double APF_FORCE_FACTOR = 0.0;
constexpr double APF_MAX_FORCE = 1.0E9;
constexpr double CUTOFF_RADIUS = 0.25;

#endif //APF_HGG_CORE_CONSTANTS_H
