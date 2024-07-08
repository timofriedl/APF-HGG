#ifndef APF_HGG_CORE_CONSTANTS_H
#define APF_HGG_CORE_CONSTANTS_H

#include <cstddef>

constexpr size_t JOINT_COUNT = 7;
constexpr size_t ACT_COUNT = 8;

constexpr double DERIVATIVE_EPSILON = 1E-5;

constexpr double APF_FORCE_FACTOR = 1000.0;
constexpr double APF_MAX_FORCE = 1E9;
constexpr double CUTOFF_RADIUS = 0.25;

constexpr double PID_MAX_FORCE = 1E9;
constexpr double ACTUAL_MAX_FORCE = 1E9;

#endif //APF_HGG_CORE_CONSTANTS_H
