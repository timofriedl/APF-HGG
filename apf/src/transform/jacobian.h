#ifndef APF_HGG_CORE_JACOBIAN_H
#define APF_HGG_CORE_JACOBIAN_H

#include <array>
#include "Eigen/Dense"
#include "../constants.h"
#include "../transform/transform.h"

namespace jacobian {
    typedef Eigen::Matrix<double, 3, JOINT_COUNT> Jacobian;
    typedef std::array<Jacobian, JOINT_COUNT> Jacobians;

    Jacobians velocityJacobians(const Eigen::Vector<double, JOINT_COUNT> &theta, const transform::TMatrices &tMatrices, const transform::TMatrices &tProducts);
    Jacobians orientationJacobians(const transform::TMatrices &tProducts);
}

#endif //APF_HGG_CORE_JACOBIAN_H
