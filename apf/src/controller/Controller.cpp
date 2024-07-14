#include "Controller.h"
#include "../transform/transform.h"
#include "../transform/jacobian.h"
#include "../apf/apf.h"

Eigen::Vector<double, JOINT_COUNT> limitAbs(const Eigen::Vector<double, JOINT_COUNT> &v, double maxAbsValue) {
    return v.cwiseMin(maxAbsValue).cwiseMax(-maxAbsValue);
}

Eigen::Vector<double, JOINT_COUNT> Controller::update() {
    // Compute transform matrices
    auto [tMatrices, tProducts] = transform::createTransformMatrices(theta);
    assert(tMatrices.size() == JOINT_COUNT + 1);
    assert(tProducts.size() == JOINT_COUNT + 1);

    // Compute inverse transform matrices
    transform::TMatrices tProductsInv = transform::invertTransformMatrices(tProducts);

    // Compute error
    Eigen::Vector3d currentPos = (tProducts[tProducts.size() - 1] * Eigen::Vector4d(0, 0, 0, 1)).head<3>();
    Eigen::Vector3d taskError = targetPos - currentPos;

    jacobian::Jacobians vJacobians = jacobian::velocityJacobians(theta, tMatrices, tProducts);
    jacobian::Jacobians oJacobians = jacobian::orientationJacobians(tProducts);

    jacobian::Jacobian eefJacobian = vJacobians[JOINT_COUNT - 1];
    Eigen::Vector<double, JOINT_COUNT> error = eefJacobian.transpose() * taskError;

    // Compute PID values
    integral += error * dt;
    const Eigen::Vector<double, JOINT_COUNT> p = error * K_p;
    const Eigen::Vector<double, JOINT_COUNT> i = integral * K_i;
    const Eigen::Vector<double, JOINT_COUNT> d = (error - prevError) * (K_d / dt);
    prevError = error;

    Eigen::Vector<double, JOINT_COUNT> forces = limitAbs(p + i + d, PID_MAX_FORCE);

    // Compute APF torques
    auto apfTorques = apf::computeTorques(tProducts, tProductsInv, vJacobians, oJacobians, obstacles);

    // Add torques
    forces += apfTorques;
    return forces;
}
