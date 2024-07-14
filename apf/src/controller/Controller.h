#ifndef APF_HGG_CORE_CONTROLLER_H
#define APF_HGG_CORE_CONTROLLER_H


#include <array>
#include <Eigen/Dense>
#include "../geometry/Capsule.h"
#include "../constants.h"

class Controller {
    const Eigen::Vector<double, JOINT_COUNT> &theta;
    const Eigen::Vector3d &targetPos;
    const Eigen::Quaterniond &targetRot;
    const std::vector<Capsule> &obstacles;
    Eigen::Vector<double, JOINT_COUNT> &integral;
    Eigen::Vector<double, JOINT_COUNT> &prevError;
    const double dt;

    static Eigen::Vector<double, JOINT_COUNT> limitAbs(const Eigen::Vector<double, JOINT_COUNT> &v, double maxAbsValue);

    Eigen::Vector<double, JOINT_COUNT> computePidForces(const Eigen::Vector<double, JOINT_COUNT> &error);

public:
    Controller(const Eigen::Vector<double, JOINT_COUNT> &theta,
               const Eigen::Vector3d &targetPos, const Eigen::Quaterniond &targetRot,
               const std::vector<Capsule> &obstacles, Eigen::Vector<double, JOINT_COUNT> &integral,
               Eigen::Vector<double, JOINT_COUNT> &prevError, double dt)
            : theta(theta),
              targetPos(targetPos),
              targetRot(targetRot),
              obstacles(obstacles),
              integral(integral),
              prevError(prevError),
              dt(dt) {};

    Eigen::Vector<double, JOINT_COUNT> update();
};


#endif //APF_HGG_CORE_CONTROLLER_H
