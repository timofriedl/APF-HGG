#include "jacobian.h"
#include <iostream>

namespace jacobian {
    Jacobians
    velocityJacobians(const Eigen::Vector<double, JOINT_COUNT> &theta, const transform::TMatrices &tMatrices,
                      const transform::TMatrices &tProducts) {
        assert(tMatrices.size() == JOINT_COUNT + 1);
        assert(tProducts.size() == JOINT_COUNT + 1);

        // Compute sin and cos values for modified theta vectors
        std::array<double, JOINT_COUNT> cos{}, sin{};
        for (Eigen::Index j = 0; j < JOINT_COUNT; j++) {
            double thetaDash = theta[j] + DERIVATIVE_EPSILON;
            cos[j] = std::cos(thetaDash);
            sin[j] = std::sin(thetaDash);
        }

        // Compute modified transformation matrices
        transform::TMatrices modifiedTMatrices;
        assert(modifiedTMatrices.size() == JOINT_COUNT + 1);
        for (Eigen::Index j = 0; j < JOINT_COUNT; j++)
            modifiedTMatrices[j] = transform::createTransformMatrix(j, cos[j], sin[j]);

        // Build jacobians
        Jacobians jacobians;
        assert(jacobians.size() == JOINT_COUNT);
        for (long i = 0; i < JOINT_COUNT; i++) {
            // Start with a zero matrix
            jacobians[i] = Jacobian::Zero();

            // Determine the position of the i-th frame in the zero frame
            Eigen::Vector3d zeroPoi = tProducts[i].block<3, 1>(0, 3);

            // Fill the jacobian up to the i-th column
            for (long j = 0; j <= i; j++) {
                transform::TMatrix modifiedTProduct = modifiedTMatrices[j];
                if (j != 0)
                    modifiedTProduct = tProducts[j - 1] * modifiedTProduct;

                for (long k = j + 1; k <= i; k++)
                    modifiedTProduct *= tMatrices[k];

                // Position of the i-th frame in the zero frame, if the j-th joint has been moved slightly
                Eigen::Vector3d zeroPoiDash = modifiedTProduct.block<3, 1>(0, 3);
                Eigen::Vector3d jColumn = (zeroPoiDash - zeroPoi) * (1.0 / DERIVATIVE_EPSILON);
                jacobians[i].block<3, 1>(0, j) = jColumn;
            }
        }
        return jacobians;
    }
}

jacobian::Jacobians jacobian::orientationJacobians(const transform::TMatrices &tProducts) {
    assert(tProducts.size() == JOINT_COUNT + 1);

    Jacobians jacobians;
    assert(jacobians.size() == JOINT_COUNT);
    for (Eigen::Index i = 0; i < JOINT_COUNT; i++)
        jacobians[i] = Jacobian::Zero();

    for (Eigen::Index i = 0; i < JOINT_COUNT; i++) {
        auto& z = tProducts[i].block<3, 1>(0, 2);
        for (Eigen::Index j = i; j < JOINT_COUNT; j++)
            jacobians[j].block<3, 1>(0, i) = z;
    }

    return jacobians;
}
