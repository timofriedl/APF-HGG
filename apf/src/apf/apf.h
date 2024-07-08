#ifndef APF_HGG_CORE_APF_H
#define APF_HGG_CORE_APF_H

#include <Eigen/Dense>
#include "../constants.h"
#include "../geometry/Capsule.h"
#include "../transform/transform.h"
#include "../transform/jacobian.h"

namespace apf {
    std::vector<std::pair<size_t, Capsule>> robotCapsules(const transform::TMatrices &tProducts);

    Eigen::Vector<double, JOINT_COUNT>
    computeTorque(const Capsule &linkCapsule, const Capsule &obstacle, const transform::TMatrix &tProductInv,
                  const jacobian::Jacobian &vJacobian, const jacobian::Jacobian &oJacobian);

    Eigen::Vector<double, JOINT_COUNT>
    computeTorques(const transform::TMatrices &tProducts, const transform::TMatrices &tProductsInv,
                   const jacobian::Jacobians &vJacobians, const jacobian::Jacobians &oJacobians,
                   const std::vector<Capsule> &obstacles);
}

#endif //APF_HGG_CORE_APF_H
