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

    // Compute jacobians
    jacobian::Jacobians vJacobians = jacobian::velocityJacobians(theta, tMatrices, tProducts);
    jacobian::Jacobians oJacobians = jacobian::orientationJacobians(tProducts);

    // Compute positional error
    Eigen::Vector3d currentEefPos = (tProducts[tProducts.size() - 1] * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
    Eigen::Vector3d taskError = targetPos - currentEefPos;
    std::cout << taskError.norm() << "\n";

    // Limit task space error to avoid large joint velocities near singularities
    if (taskError.squaredNorm() > MAX_TASK_SPACE_VELOCITY * MAX_TASK_SPACE_VELOCITY)
        taskError = taskError.normalized() * MAX_TASK_SPACE_VELOCITY;

    // Compute joint space error using Jacobian damping
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(vJacobians[JOINT_COUNT - 1], Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd damped_inverse = svd.matrixV() * (svd.singularValues().array() / (svd.singularValues().array().square() + JACOBIAN_DAMP_FACTOR * JACOBIAN_DAMP_FACTOR)).matrix().asDiagonal() * svd.matrixU().transpose();
    Eigen::Vector<double, JOINT_COUNT> posJointError = damped_inverse * taskError;

    // Compute orientation error
    Eigen::Quaterniond currentRot(tProducts[JOINT_COUNT].topLeftCorner<3, 3>());
    Eigen::Quaterniond rotError = targetRot * currentRot.conjugate();
    Eigen::AngleAxisd angleAxisError(rotError);
    double angleError = angleAxisError.angle();
    Eigen::Vector3d axisError = angleAxisError.axis();

    // Compute rotational joint error
    Eigen::Vector<double, JOINT_COUNT> rotJointError = oJacobians[JOINT_COUNT - 1].transpose() * (axisError * angleError);
    Eigen::Vector<double, JOINT_COUNT> totalJointError = posJointError + rotJointError;

    // Compute PID and APF torques
    auto pidTorques = computePidForces(totalJointError);
    auto apfTorques = apf::computeTorques(tProducts, vJacobians, oJacobians, obstacles);
    return pidTorques + apfTorques;
}
