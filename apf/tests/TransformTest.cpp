#include <gtest/gtest.h>
#include "transform/jacobian.h"
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(TransformTest, CreateTransformMatrices) {
    Eigen::Vector<double, JOINT_COUNT> theta;
    theta.setConstant(M_PI);

    auto [tMatrices, tProducts] = transform::createTransformMatrices(theta);

    transform::TMatrix t0;
    t0 << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0.333,
            0, 0, 0, 1;

    transform::TMatrix t1;
    t1 << -1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix t2;
    t2 << -1, 0, 0, 0,
            0, 0, -1, -0.316,
            0, -1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix t3;
    t3 << -1, 0, 0, 0.0825,
            0, 0, -1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix t4;
    t4 << -1, 0, 0, -0.0825,
            0, 0, 1, 0.384,
            0, 1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix t5;
    t5 << -1, 0, 0, 0,
            0, 0, -1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix t6;
    t6 << -1, 0, 0, 0.088,
            0, 0, -1, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    transform::TMatrix tf;
    tf << 0.70710678118654752440084436210, 0.70710678118654752440084436210, 0, 0,
            -0.70710678118654752440084436210, 0.70710678118654752440084436210, 0, 0,
            0, 0, 1, 0.245,
            0, 0, 0, 1;

    EXPECT_TRUE(tMatrices[0].isApprox(t0));
    EXPECT_TRUE(tMatrices[1].isApprox(t1));
    EXPECT_TRUE(tMatrices[2].isApprox(t2));
    EXPECT_TRUE(tMatrices[3].isApprox(t3));
    EXPECT_TRUE(tMatrices[4].isApprox(t4));
    EXPECT_TRUE(tMatrices[5].isApprox(t5));
    EXPECT_TRUE(tMatrices[6].isApprox(t6));
    EXPECT_TRUE(tMatrices[7].isApprox(tf));

    EXPECT_TRUE(tProducts[0].isApprox(t0));
    EXPECT_TRUE(tProducts[1].isApprox(t0 * t1));
    EXPECT_TRUE(tProducts[2].isApprox(t0 * t1 * t2));
    EXPECT_TRUE(tProducts[3].isApprox(t0 * t1 * t2 * t3));
    EXPECT_TRUE(tProducts[4].isApprox(t0 * t1 * t2 * t3 * t4));
    EXPECT_TRUE(tProducts[5].isApprox(t0 * t1 * t2 * t3 * t4 * t5));
    EXPECT_TRUE(tProducts[6].isApprox(t0 * t1 * t2 * t3 * t4 * t5 * t6));
    EXPECT_TRUE(tProducts[7].isApprox(t0 * t1 * t2 * t3 * t4 * t5 * t6 * tf));

    for (int i = 0; i <= 7; i++)
        EXPECT_TRUE(tMatrices[i].isApprox(transform::createTransformMatrix(i, -1.0, 0.0)));
}

TEST(TransformTest, InvertTransformMatrices) {
    transform::TMatrices tMatrices;
    assert(tMatrices.size() == JOINT_COUNT + 1);

    for (auto &matrix: tMatrices)
        matrix << 1, 2, 3, 4,
                5, 6, 7, 8,
                9, 10, 11, 12,
                0, 0, 0, 1;

    auto inv = transform::invertTransformMatrices(tMatrices);

    transform::TMatrix expected;
    expected << 1, 5, 9, -152,
            2, 6, 10, -176,
            3, 7, 11, -200,
            0, 0, 0, 1;

    for (Eigen::Index i = 0; i < tMatrices.size(); i++)
        EXPECT_TRUE(inv[i].isApprox(expected));
}