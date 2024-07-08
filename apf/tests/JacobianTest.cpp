#include <gtest/gtest.h>
#include "transform/jacobian.h"
#include <Eigen/Dense>

TEST(JacobianTest, VelocityJacobians) {
    const auto theta = Eigen::Vector<double, JOINT_COUNT>::Zero();
    auto [tMatrices, tProducts] = transform::createTransformMatrices(theta);
    auto jacobians = jacobian::velocityJacobians(theta, tMatrices, tProducts);
    assert(jacobians.size() == JOINT_COUNT);

    auto jv0Zero = jacobians[0].block<3, 6>(0, 1);
    EXPECT_TRUE(jv0Zero.isApprox(Eigen::Matrix<double, 3, 6>::Zero()));

    auto jv1Zero = jacobians[1].block<3, 5>(0, 2);
    EXPECT_TRUE(jv1Zero.isApprox(Eigen::Matrix<double, 3, 5>::Zero()));

    auto jv2Zero = jacobians[2].block<3, 4>(0, 3);
    EXPECT_TRUE(jv2Zero.isApprox(Eigen::Matrix<double, 3, 4>::Zero()));

    auto jv3Zero = jacobians[3].block<3, 3>(0, 4);
    EXPECT_TRUE(jv3Zero.isApprox(Eigen::Matrix<double, 3, 3>::Zero()));

    auto jv4Zero = jacobians[4].block<3, 2>(0, 5);
    EXPECT_TRUE(jv4Zero.isApprox(Eigen::Matrix<double, 3, 2>::Zero()));

    auto jv5Zero = jacobians[5].block<3, 1>(0, 6);
    EXPECT_TRUE(jv5Zero.isApprox(Eigen::Matrix<double, 3, 1>::Zero()));
}

TEST(JacobianTest, OrientationJacobians) {
    const auto theta = Eigen::Vector<double, JOINT_COUNT>::Zero();
    auto [tMatrices, tProducts] = transform::createTransformMatrices(theta);
    auto jacobians = jacobian::orientationJacobians(tProducts);
    assert(jacobians.size() == JOINT_COUNT);

    auto jv0Zero = jacobians[0].block<3, 6>(0, 1);
    EXPECT_TRUE(jv0Zero.isApprox(Eigen::Matrix<double, 3, 6>::Zero()));

    auto jv1Zero = jacobians[1].block<3, 5>(0, 2);
    EXPECT_TRUE(jv1Zero.isApprox(Eigen::Matrix<double, 3, 5>::Zero()));

    auto jv2Zero = jacobians[2].block<3, 4>(0, 3);
    EXPECT_TRUE(jv2Zero.isApprox(Eigen::Matrix<double, 3, 4>::Zero()));

    auto jv3Zero = jacobians[3].block<3, 3>(0, 4);
    EXPECT_TRUE(jv3Zero.isApprox(Eigen::Matrix<double, 3, 3>::Zero()));

    auto jv4Zero = jacobians[4].block<3, 2>(0, 5);
    EXPECT_TRUE(jv4Zero.isApprox(Eigen::Matrix<double, 3, 2>::Zero()));

    auto jv5Zero = jacobians[5].block<3, 1>(0, 6);
    EXPECT_TRUE(jv5Zero.isApprox(Eigen::Matrix<double, 3, 1>::Zero()));

    Eigen::Matrix<double, 3, JOINT_COUNT> expected;
    expected << 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, -1, 0, -1, 0,
            1, 0, 1, 0, 1, 0, -1;

    EXPECT_TRUE(jacobians[6].isApprox(expected));
}