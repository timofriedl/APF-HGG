#include "apf.h"
#include <iostream>

std::vector<std::pair<size_t, Capsule>> apf::robotCapsules(const transform::TMatrices &tProducts) {
    const std::vector<std::tuple<size_t, Eigen::Vector4d, Eigen::Vector4d, double>> coordinates = {
            {0, {0,    0,     -0.333, 1}, {0,    0,      0,      1}, 0.07},  // Torso     (1. Frame)
            {1, {0,    0,     -0.055, 1}, {0,    0,      0.055,  1}, 0.075}, // Shoulder  (2. Frame)
            {2, {0,    0,     -0.064, 1}, {0,    0,      -0.316, 1}, 0.07},  // Upper arm (3. Frame)
            {3, {0,    0,     -0.055, 1}, {0,    0,      0.055,  1}, 0.075}, // Elbow     (4. Frame)
            {4, {0,    0,     -0.23,  1}, {0,    0,      -0.32,  1}, 0.07},  // Forearm 1 (5. Frame)
            {4, {0,    0.07,  -0.18,  1}, {0,    0.07,   0,      1}, 0.045}, // Forearm 2 (5. Frame)
            {5, {0,    0,     -0.08,  1}, {0,    0,      0.01,   1}, 0.067}, // Wrist     (6. Frame)
            {6, {0,    0,     -0.15,  1}, {0,    0,      0.175,  1}, 0.065}, // Hand 1    (7. Frame)
            {7, {0,    0.061, -0.115, 1}, {0,    -0.061, -0.115, 1}, 0.065}, // Hand 2    (Flange Frame)
            {6, {0.03, 0.06,  0.085,  1}, {0.06, 0.03,   0.085,  1}, 0.035}  // Hand 3    (7. Frame)
    };

    std::vector<std::pair<size_t, Capsule>> robotCapsules;
    robotCapsules.reserve(coordinates.size());
    for (auto &[linkId, fromPoint, toPoint, radius]: coordinates) {
        assert(linkId <= JOINT_COUNT);
        assert(fromPoint[3] == 1);
        assert(toPoint[3] == 1);
        assert(radius > 0);
        Eigen::Vector3d from = (tProducts[linkId] * fromPoint).head<3>();
        Eigen::Vector3d to = (tProducts[linkId] * toPoint).head<3>();
        Line line = {std::move(from), std::move(to)};

        /* Debugging output

        Eigen::Vector3d offset{0.8, 0.75, 0.44};
        auto f = line.getFrom() + offset;
        auto t = line.getTo() + offset;
        auto d = t - f;
        auto halfLength = 0.5 * d.norm();
        auto p = 0.5 * (t + f);
        Eigen::Vector3d zAxis(0.0, 0.0, 1.0);
        Eigen::Quaterniond quaternion = Eigen::Quaterniond::FromTwoVectors(zAxis, d);
        auto rotMatrix = quaternion.toRotationMatrix();
        auto rot = rotMatrix.eulerAngles(0, 1, 2);

        std::cout << "<body name=\"capsule" << i << "\" pos=\"0 0 0\">\n"
            << "    <geom type=\"capsule\" name=\"capsule" << i << ":geom\" size=\"" << radius << " " << halfLength
            << "\" pos=\"" << p[0] << " " << p[1] << " " << p[2]
            << "\" euler=\"" << rot[0] << " " << rot[1] << " " << rot[2] << "\" rgba=\"1 0 0 0.2\"/>\n"
            << "</body>\n";

        i++; */

        robotCapsules.push_back({linkId, {std::move(line), radius + APF_MARGIN}});
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
    const Eigen::Vector3d forceVector = (line.getFrom() - act) * forceMagnitude / dis;

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
    Eigen::Vector<double, JOINT_COUNT> torque = poiJacobian.transpose() * forceVector;

    // Replace NaN values
    for (Eigen::Index i = 0; i < JOINT_COUNT; i++)
        if (std::isnan(torque[i]))
            torque[i] = 0.0;

    return torque;
}

Eigen::Vector<double, JOINT_COUNT>
apf::computeTorques(const transform::TMatrices &tProducts, const jacobian::Jacobians &vJacobians,
                    const jacobian::Jacobians &oJacobians,
                    const std::vector<Capsule> &obstacles) {
    transform::TMatrices tProductsInv = transform::invertTransformMatrices(tProducts);
    Eigen::Vector<double, JOINT_COUNT> torques = Eigen::Vector<double, JOINT_COUNT>::Zero();

    const auto robotCapsules = apf::robotCapsules(tProducts);
    for (auto &[i, linkCapsule]: robotCapsules) {
        // Obstacle collision
        for (auto &obstacle: obstacles)
            torques += apf::computeTorque(linkCapsule, obstacle, tProductsInv[i], vJacobians[i], oJacobians[i]);

        // Self-collision
        for (auto &[j, otherLinkCapsule]: robotCapsules)
            if (j < i - 2 || j > i + 2) // Don't check self-collision with closely neighbored links
                torques += apf::computeTorque(linkCapsule, otherLinkCapsule, tProductsInv[i], vJacobians[i], oJacobians[i]);
    }

    return torques;
}
