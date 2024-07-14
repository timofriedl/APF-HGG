#include "apf.h"
#include <iostream>

std::vector<std::pair<size_t, Capsule>> apf::robotCapsules(const transform::TMatrices &tProducts) {
    std::vector<std::tuple<size_t, Eigen::Vector4d, Eigen::Vector4d, double>> coordinates = {
            {3, {0,    0,     -0.055, 1}, {0,    0,      0.055, 1}, 0.075}, // Elbow (4. Frame)
            {4, {0,    0,     -0.23,  1}, {0,    0,      -0.32, 1}, 0.07}, // Forearm 1 (5. Frame)
            // {4, {0, 0.07, -0.18, 1}, 5, {0, 0, -0.1, 1}, 0.045}, // Forearm 2 (5. & 6. Frame) TODO tf model this using only one link and not two
            {5, {0,    0,     -0.08,  1}, {0,    0,      0.01,  1}, 0.067}, // Wrist (6. Frame)
            {6, {0,    0,     -0.04,  1}, {0,    0,      0.175, 1}, 0.065}, // Hand 1 (7. Frame)
            {6, {0,    0.061, 0.13,   1}, {0,    -0.061, 0.13,  1}, 0.065}, // Hand 2 (7. Frame)
            {6, {0.03, 0.06,  0.085,  1}, {0.06, 0.03,   0.085, 1}, 0.035} // Hand 3 (7. Frame)
    };

    std::vector<std::pair<size_t, Capsule>> robotCapsules;
    robotCapsules.reserve(coordinates.size());
    for (auto &[linkId, fromPoint, toPoint, radius]: coordinates) {
        assert(linkId < JOINT_COUNT);
        assert(fromPoint[3] == 1);
        assert(toPoint[3] == 1);
        assert(radius > 0);
        Eigen::Vector3d from = (tProducts[linkId] * fromPoint).head<3>();
        Eigen::Vector3d to = (tProducts[linkId] * toPoint).head<3>();
        Line line = {std::move(from), std::move(to)};
        robotCapsules.push_back({linkId, {std::move(line), radius}});
    }

    return robotCapsules;
}

Eigen::Vector<double, JOINT_COUNT>
apf::computeTorque(const Capsule &linkCapsule, const Capsule &obstacle, const transform::TMatrix &tProductInv,
                   const jacobian::Jacobian &vJacobian, const jacobian::Jacobian &oJacobian) {
    auto [line, dis] = obstacle.shortestLineFrom(linkCapsule);

    if (dis >= CUTOFF_RADIUS)
        return Eigen::Vector<double, JOINT_COUNT>::Zero();

    // Compute the magnitude of the repulsive force that decreases quadratically with distance
    double forceMagnitude = dis <= std::numeric_limits<double>::epsilon()
                            ? APF_MAX_FORCE
                            : APF_FORCE_FACTOR / (dis * dis);

    assert(forceMagnitude >= 0);
    if (forceMagnitude > APF_MAX_FORCE)
        forceMagnitude = APF_MAX_FORCE;

    // Compute the force vector acting on the border of the link capsule.
    // 'dis' may be negative if collisions occur. It then reverses the incorrectly directed vector to restore it as a repulsive vector.
    const Eigen::Vector3d &act = line.getTo();
    const Eigen::Vector3d forceVector = (act - line.getFrom()) * forceMagnitude / dis;

    // Position of acting force in the frame i
    const Eigen::Vector3d v = (tProductInv * Eigen::Vector4d(act.x(), act.y(), act.z(), 1)).head<3>();

    // Build skew-symmetric matrix of offset vector
    Eigen::Matrix3d frameOffsetSkewMatrix;
    frameOffsetSkewMatrix << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;

    // Velocity jacobian with respect to the acting force point
    const jacobian::Jacobian poiJacobian = vJacobian + frameOffsetSkewMatrix * oJacobian;

    // Compute the joint-space force (torque), given the task-space force and the velocity jacobian
    return poiJacobian.transpose() * forceVector;
}

Eigen::Vector<double, JOINT_COUNT>
apf::computeTorques(const transform::TMatrices &tProducts, const jacobian::Jacobians &vJacobians,
                    const jacobian::Jacobians &oJacobians,
                    const std::vector<Capsule> &obstacles) {
    transform::TMatrices tProductsInv = transform::invertTransformMatrices(tProducts);
    Eigen::Vector<double, JOINT_COUNT> torques = Eigen::Vector<double, JOINT_COUNT>::Zero();

    for (auto &[i, linkCapsule]: apf::robotCapsules(tProducts)) {
        for (auto &obstacle: obstacles) {
            torques += apf::computeTorque(linkCapsule, obstacle, tProductsInv[i], vJacobians[i], oJacobians[i]);
        }

        // TODO tf self-collision
    }

    return torques;
}
