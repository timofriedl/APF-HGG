#include <gtest/gtest.h>
#include "geometry/Capsule.h"
#include <Eigen/Dense>

TEST(CapsuleTest, ShortestLineLinear) {
    Capsule c1({{1, 0, 0}, {10, 0, 0}}, 2.0);
    Capsule c2({{20, 0, 0}, {30, 0, 0}}, 1.0);
    auto [line, dis] = c1.shortestLineFrom(c2);

    EXPECT_NEAR(7.0, dis, 1E-10);

    EXPECT_TRUE(line.getFrom().isApprox(Eigen::Vector3d(19, 0, 0)));
    EXPECT_TRUE(line.getTo().isApprox(Eigen::Vector3d(12, 0, 0)));
}

TEST(CapsuleTest, ShortestLineOrthogonal) {
    Capsule c1({{1, 0, 0}, {10, 0, 0}}, 2.0);
    Capsule c2({{5, 100, 0}, {5, 5, 0}}, 1.0);
    auto [line, dis] = c1.shortestLineFrom(c2);

    EXPECT_NEAR(2.0, dis, 1E-10);

    EXPECT_TRUE(line.getFrom().isApprox(Eigen::Vector3d(5, 4, 0)));
    EXPECT_TRUE(line.getTo().isApprox(Eigen::Vector3d(5, 2, 0)));
}

TEST(CapsuleTest, ShortestLineCross) {
    Capsule c1({{1, 0, 0}, {10, 0, 0}}, 2.0);
    Capsule c2({{5, 100, 0}, {5, -5, 0}}, 1.0);
    auto [line, dis] = c1.shortestLineFrom(c2);

    EXPECT_TRUE(dis > 1E100);

    EXPECT_TRUE(line.getFrom().isApprox(Eigen::Vector3d(5, 0, 0)));
    EXPECT_TRUE(line.getTo().isApprox(Eigen::Vector3d(5, 0, 0)));
}

TEST(CapsuleTest, ShortestLineOverlap) {
    Capsule c1({{1, 0, 0}, {10, 0, 0}}, 2.0);
    Capsule c2({{-10, 2.5, 0}, {20, 2.5, 0}}, 1.0);
    auto [line, dis] = c1.shortestLineFrom(c2);

    EXPECT_NEAR(-0.5, dis, 1E-10);

    EXPECT_TRUE((line.getTo() - line.getFrom()).isApprox(Eigen::Vector3d(0, 0.5, 0)));
}