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

    // Compute orientation error
    Eigen::Quaterniond currentRot(tProducts[JOINT_COUNT].topLeftCorner<3, 3>());
    Eigen::Quaterniond rotError = targetRot * currentRot.conjugate();
    rotError.normalize();
    Eigen::AngleAxisd angleAxisError(rotError);
    Eigen::Vector3d taskErrorRot = angleAxisError.axis() * (angleAxisError.angle() * rotWeight);

    // Compute the pseudo-inverse of the orientation Jacobian
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOrient(oJacobians[JOINT_COUNT - 1], Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double, JOINT_COUNT, 3> orientPseudoInverse = svdOrient.matrixV() *
        (svdOrient.singularValues().array() /
         (svdOrient.singularValues().array().square() + JACOBIAN_DAMP_FACTOR * JACOBIAN_DAMP_FACTOR))
        .matrix().asDiagonal() * svdOrient.matrixU().transpose();

    // Joint-space correction for orientation task
    Eigen::Vector<double, JOINT_COUNT> jointErrorOrient = orientPseudoInverse * taskErrorRot;

    // Compute the null space projection matrix for the orientation task
    Eigen::Matrix<double, JOINT_COUNT, JOINT_COUNT> nullSpaceProjection =
        Eigen::Matrix<double, JOINT_COUNT, JOINT_COUNT>::Identity() - orientPseudoInverse * oJacobians[JOINT_COUNT - 1];

    // Compute the pseudo-inverse of the velocity Jacobian for positional control
    Eigen::JacobiSVD<Eigen::MatrixXd> svdPos(vJacobians[JOINT_COUNT - 1], Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double, JOINT_COUNT, 3> posPseudoInverse = svdPos.matrixV() *
        (svdPos.singularValues().array() /
         (svdPos.singularValues().array().square() + JACOBIAN_DAMP_FACTOR * JACOBIAN_DAMP_FACTOR))
        .matrix().asDiagonal() * svdPos.matrixU().transpose();

    // Apply null space projection to positional correction
    Eigen::Vector<double, JOINT_COUNT> jointErrorPos = nullSpaceProjection * (posPseudoInverse * taskErrorPos);

    // Combine orientation and strictly null-space positional errors
    Eigen::Vector<double, JOINT_COUNT> totalJointError = jointErrorOrient + jointErrorPos;

    // Compute torques
    auto pidTorques = computePidForces(totalJointError);
    auto apfTorques = apf::computeTorques(tProducts, vJacobians, oJacobians, obstacles);
    return pidTorques + apfTorques;
}
