#include "transform.h"

namespace transform {
    inline TMatrix transform0_1(double c0, double s0) {
        TMatrix t;
        t << c0, -s0, 0, 0,
                s0, c0, 0, 0,
                0, 0, 1, 0.333,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform1_2(double c1, double s1) {
        TMatrix t;
        t << c1, -s1, 0, 0,
                0, 0, 1, 0,
                -s1, -c1, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform2_3(double c2, double s2) {
        TMatrix t;
        t << c2, -s2, 0, 0,
                0, 0, -1, -0.316,
                s2, c2, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform3_4(double c3, double s3) {
        TMatrix t;
        t << c3, -s3, 0, 0.0825,
                0, 0, -1, 0,
                s3, c3, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform4_5(double c4, double s4) {
        TMatrix t;
        t << c4, -s4, 0, -0.0825,
                0, 0, 1, 0.384,
                -s4, -c4, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform5_6(double c5, double s5) {
        TMatrix t;
        t << c5, -s5, 0, 0,
                0, 0, -1, 0,
                s5, c5, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform6_7(double c6, double s6) {
        TMatrix t;
        t << c6, -s6, 0, 0.088,
                0, 0, -1, 0,
                s6, c6, 0, 0,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix transform_flange() {
        constexpr double SQRT_2_HALF = 0.70710678118654752440084436210;
        TMatrix t;
        t << SQRT_2_HALF, SQRT_2_HALF, 0, 0,
                -SQRT_2_HALF, SQRT_2_HALF, 0, 0,
                0, 0, 1, 0.245,
                0, 0, 0, 1;
        return t;
    }

    inline TMatrix createTransformMatrix(size_t i, double ci, double si) {
        switch (i) {
            case 0:
                return transform0_1(ci, si);
            case 1:
                return transform1_2(ci, si);
            case 2:
                return transform2_3(ci, si);
            case 3:
                return transform3_4(ci, si);
            case 4:
                return transform4_5(ci, si);
            case 5:
                return transform5_6(ci, si);
            case 6:
                return transform6_7(ci, si);
            case 7:
                return transform_flange();
            default:
                assert(false);
        }
    }

    std::pair<TMatrices, TMatrices> createTransformMatrices(const Eigen::Vector<double, JOINT_COUNT> &theta) {
        assert(JOINT_COUNT == 7);

        // Compute sin and cos values
        std::array<double, JOINT_COUNT> cos{}, sin{};
        for (Eigen::Index i = 0; i < JOINT_COUNT; i++) {
            cos[i] = std::cos(theta[i]);
            sin[i] = std::sin(theta[i]);
        }

        // Compute transformation matrices
        TMatrices matrices;
        assert(matrices.size() == JOINT_COUNT + 1);
        for (size_t i = 0; i < JOINT_COUNT; i++)
            matrices[i] = createTransformMatrix(i, cos[i], sin[i]);

        matrices[7] = transform_flange();

        // Multiply matrices
        TMatrices product;
        assert(product.size() == JOINT_COUNT + 1);
        product[0] = matrices[0];
        for (Eigen::Index i = 1; i < JOINT_COUNT + 1; i++)
            product[i] = product[i - 1] * matrices[i];

        return {matrices, product};
    }

    TMatrices invertTransformMatrices(const transform::TMatrices &tMatrices) {
        TMatrices result;
        for (Eigen::Index i = 0; i < tMatrices.size(); i++) {
            auto &rotationT = tMatrices[i].block<3, 3>(0, 0).transpose();
            result[i].block<3, 3>(0, 0) = rotationT;
            result[i].block<3, 1>(0, 3) = -(rotationT * tMatrices[i].block<3, 1>(0, 3));
            result[i].block<1, 3>(3, 0) = Eigen::Matrix<double, 1, 3>::Zero();
            result[i].coeffRef(3, 3) = 1;
        }
        return result;
    }
} // namespace transform