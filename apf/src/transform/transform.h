#ifndef APF_HGG_CORE_TRANSFORM_H
#define APF_HGG_CORE_TRANSFORM_H

#include <array>
#include "Eigen/Dense"
#include "../constants.h"

namespace transform {
    typedef Eigen::Matrix4d TMatrix;
    typedef std::array<TMatrix, JOINT_COUNT + 1> TMatrices;

    TMatrix createTransformMatrix(size_t i, double ci, double si);

    std::pair<TMatrices, TMatrices> createTransformMatrices(const Eigen::Vector<double, JOINT_COUNT>& theta);

    TMatrices invertTransformMatrices(const TMatrices &tMatrices);
}

#endif //APF_HGG_CORE_TRANSFORM_H
