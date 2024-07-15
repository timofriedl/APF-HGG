#ifndef APF_HGG_CORE_CONSTANTS_H
#define APF_HGG_CORE_CONSTANTS_H

#include <cstddef>

// Robot
constexpr size_t JOINT_COUNT = 7;

// Jacobian
constexpr double DERIVATIVE_EPSILON = 1E-5;

// PID Controller
constexpr double K_p = 1000.0;
constexpr double K_i = 0.0;
constexpr double K_d = 0.0; // TODO tf tune

constexpr double PID_MAX_FORCE = 1.0E9;

// APF
constexpr double APF_FORCE_FACTOR = 0.0;
constexpr double APF_MAX_FORCE = 1.0E9;
constexpr double CUTOFF_RADIUS = 0.25;

#endif //APF_HGG_CORE_CONSTANTS_H
