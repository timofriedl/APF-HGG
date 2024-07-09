#ifndef APF_HGG_CORE_CONTROLLER_H
#define APF_HGG_CORE_CONTROLLER_H


#include <array>
#include <Eigen/Dense>
#include "../geometry/Capsule.h"
#include "../constants.h"

class Controller {
    const Eigen::Vector<double, JOINT_COUNT> &theta;
    const double currentGripper;
    const Eigen::Vector3d &targetPos;
    const double targetGripper;
    const std::vector<Capsule> &obstacles;
    Eigen::Vector<double, ACT_COUNT> &integral;
    Eigen::Vector<double, ACT_COUNT> &prevError;
    const double dt;

public:
    Controller(const Eigen::Vector<double, JOINT_COUNT> &theta, double currentGripper, const Eigen::Vector3d &targetPos,
               double targetGripper, const std::vector<Capsule> &obstacles, Eigen::Vector<double, ACT_COUNT> &integral,
               Eigen::Vector<double, ACT_COUNT> &prevError, double dt)
            : theta(theta),
              currentGripper(currentGripper),
              targetPos(targetPos),
              targetGripper(targetGripper),
              obstacles(obstacles),
              integral(integral),
              prevError(prevError),
              dt(dt) {};

    Eigen::Vector<double, ACT_COUNT> update();
};


#endif //APF_HGG_CORE_CONTROLLER_H
