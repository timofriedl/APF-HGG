#ifndef APF_HGG_CORE_CONTROLLER_H
#define APF_HGG_CORE_CONTROLLER_H


#include <array>
#include <Eigen/Dense>
#include "../geometry/Capsule.h"
#include "../constants.h"

class Controller {
    const Eigen::Vector<double, JOINT_COUNT> &theta;
    const Eigen::Vector3d &targetPos;
    const std::vector<Capsule> &obstacles;
    Eigen::Vector<double, JOINT_COUNT> &integral;
    Eigen::Vector<double, JOINT_COUNT> &prevError;
    const double dt;

public:
    Controller(const Eigen::Vector<double, JOINT_COUNT> &theta, const Eigen::Vector3d &targetPos,
               const std::vector<Capsule> &obstacles, Eigen::Vector<double, JOINT_COUNT> &integral,
               Eigen::Vector<double, JOINT_COUNT> &prevError, double dt)
            : theta(theta),
              targetPos(targetPos),
              obstacles(obstacles),
              integral(integral),
              prevError(prevError),
              dt(dt) {};

    Eigen::Vector<double, JOINT_COUNT> update();
};


#endif //APF_HGG_CORE_CONTROLLER_H
