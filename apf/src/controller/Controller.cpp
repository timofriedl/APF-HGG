#include "Controller.h"
#include "../transform/transform.h"
#include "../transform/jacobian.h"
#include "../apf/apf.h"
#include <iostream>

inline Eigen::Vector<double, JOINT_COUNT> Controller::computePidForces(const Eigen::Vector<double, JOINT_COUNT> &error) {
    integral += error * dt;
    const Eigen::Vector<double, JOINT_COUNT> p = error * K_p;
    const Eigen::Vector<double, JOINT_COUNT> i = integral * K_i;
    const Eigen::Vector<double, JOINT_COUNT> d = (error - prevError) * (K_d / dt);
    prevError = error;

    Eigen::Vector<double, JOINT_COUNT> forces = p + i + d;

    // Clamp each element of forces to the range [PID_MIN_FORCE, PID_MAX_FORCE]
    for (Eigen::Index j = 0; j < JOINT_COUNT; j++)
        forces[j] = std::max(PID_MIN_FORCE[j], std::min(forces[j], PID_MAX_FORCE[j]));

    return forces;
}

Eigen::Vector<double, JOINT_COUNT> Controller::update() {
    // Compute transform matrices
    auto [tMatrices, tProducts] = transform::createTransformMatrices(theta);
    assert(tMatrices.size() == JOINT_COUNT + 1);
    assert(tProducts.size() == JOINT_COUNT + 1);

    // Compute Jacobians
    jacobian::Jacobians vJacobians = jacobian::velocityJacobians(theta, tMatrices, tProducts);
    jacobian::Jacobians oJacobians = jacobian::orientationJacobians(tProducts);

    // Compute positional error
    Eigen::Vector3d currentEefPos = (tProducts[tProducts.size() - 1] * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
    Eigen::Vector3d taskErrorPos = targetPos - currentEefPos;

    // Limit task space error to avoid large joint velocities near singularities
    if (taskErrorPos.squaredNorm() > MAX_TASK_SPACE_VELOCITY * MAX_TASK_SPACE_VELOCITY)
        taskErrorPos = taskErrorPos.normalized() * MAX_TASK_SPACE_VELOCITY;

    // Compute orientation error
    Eigen::Quaterniond currentRot(tProducts[JOINT_COUNT].topLeftCorner<3, 3>());
    Eigen::Quaterniond rotError = targetRot * currentRot.conjugate();
    rotError.normalize();
    Eigen::AngleAxisd angleAxisError(rotError);
    Eigen::Vector3d taskErrorRot = angleAxisError.axis() * angleAxisError.angle();

    // Combine position and orientation errors into a single task-space error vector (6D)
    Eigen::Matrix<double, 6, 1> taskError;
    taskError.head<3>() = taskErrorPos; // Positional error (first 3 elements)
    taskError.tail<3>() = taskErrorRot; // Orientation error (last 3 elements)

    // Create a task-space weighting matrix to balance position and orientation
    Eigen::Matrix<double, 6, 6> weightingMatrix = Eigen::Matrix<double, 6, 6>::Identity();
    weightingMatrix.block<3, 3>(3, 3) *= rotWeight;  // Scale orientation error

    // Apply the weighting matrix to the task-space error
    Eigen::Matrix<double, 6, 1> weightedTaskError = weightingMatrix * taskError;

    // Create a full jacobian
    Eigen::Matrix<double, 6, JOINT_COUNT> fullJacobian;
    fullJacobian.topRows(3) = vJacobians[JOINT_COUNT - 1];
    fullJacobian.bottomRows(3) = oJacobians[JOINT_COUNT - 1];

    // Compute the damped pseudo-inverse of the Jacobian for task-space to joint-space projection
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(vJacobians[JOINT_COUNT - 1], Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double, JOINT_COUNT, 6> dampedInverse = svd.matrixV() * (svd.singularValues().array() /
                        (svd.singularValues().array().square() + JACOBIAN_DAMP_FACTOR * JACOBIAN_DAMP_FACTOR))
                        .matrix().asDiagonal() * svd.matrixU().transpose();

    // Map weighted task-space error into joint-space
    Eigen::Vector<double, JOINT_COUNT> jointError = dampedInverse * weightedTaskError;

    // Compute PID torques based on the joint-space error
    auto pidTorques = computePidForces(jointError);

    // Compute APF (Artificial Potential Field) torques for obstacle avoidance
    auto apfTorques = apf::computeTorques(tProducts, vJacobians, oJacobians, obstacles);

    // Combine the computed torques
    return pidTorques + apfTorques;
}
