#include "constants.h"
#include "controller/Controller.h"

Capsule parseCapsule(const double *capsuleAttributes) {
    Eigen::Vector3d from(capsuleAttributes[0], capsuleAttributes[1], capsuleAttributes[2]);
    Eigen::Vector3d to(capsuleAttributes[3], capsuleAttributes[4], capsuleAttributes[5]);
    double radius = capsuleAttributes[6];

    Line line(std::move(from), std::move(to));
    return {std::move(line), radius};
}

extern "C" {
void step(const double jointAngles[JOINT_COUNT], const double targetPosValues[3], const double targetRotValues[4],
          const double *obstacleAttributes, size_t obstacleCount, double dt,
          double integralValues[JOINT_COUNT], double prevErrorValues[JOINT_COUNT],
          double forceResult[JOINT_COUNT]) {

    // Extract objects
    const Eigen::Vector<double, JOINT_COUNT> theta(jointAngles);
    const Eigen::Vector3d targetPos(targetPosValues);
    const Eigen::Quaterniond targetRot(targetRotValues);
    Eigen::Vector<double, JOINT_COUNT> integral(integralValues);
    Eigen::Vector<double, JOINT_COUNT> prevError(prevErrorValues);

    // Build obstacles
    std::vector<Capsule> obstacles;
    obstacles.reserve(obstacleCount);
    constexpr size_t numPerCapsule = 7;
    for (int i = 0; i < obstacleCount; i++)
        obstacles.push_back(std::move(parseCapsule(obstacleAttributes + i * numPerCapsule)));

    // Create controller
    Controller controller(theta, targetPos, targetRot, obstacles, integral, prevError, dt);

    // Compute
    Eigen::Map<Eigen::Vector<double, JOINT_COUNT>>(forceResult).noalias() = controller.update();

    // Update integral and prevError values
    std::copy(integral.data(), integral.data() + JOINT_COUNT, integralValues);
    std::copy(prevError.data(), prevError.data() + JOINT_COUNT, prevErrorValues);
}
}