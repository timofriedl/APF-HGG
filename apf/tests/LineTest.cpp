#include <gtest/gtest.h>
#include "geometry/Line.h"

TEST(LineTest, Constructor) {
    Eigen::Vector3d from(1, 2, 3);
    Eigen::Vector3d to(4, 5, 6);
    Line line(std::move(from), std::move(to));

    EXPECT_EQ(line.getFrom(), Eigen::Vector3d(1, 2, 3));
    EXPECT_EQ(line.getTo(), Eigen::Vector3d(4, 5, 6));
}

TEST(LineTest, ShortestLineEndToEnd) {
    Line line1({0, 0, 0}, {1, 0, 0});
    Line line2({1, 1, 0}, {2, 1, 0});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(1, 1, 0));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(1, 0, 0));
}

TEST(LineTest, ShortestLineMidToEnd) {
    Line line1({0, 0, 0}, {2, 0, 0});
    Line line2({1, 1, 0}, {1, 2, 0});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(1, 1, 0));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(1, 0, 0));
}

TEST(LineTest, ShortestLineCross) {
    Line line1({0, 0, 0}, {2, 0, 0});
    Line line2({1, 1, 0}, {1, -1, 0});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(1, 0, 0));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(1, 0, 0));
}

TEST(LineTest, ShortestLineEqual) {
    Line line1({2, 3, 5}, {7, 11, 13});
    Line line2({2, 3, 5}, {7, 11, 13});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(2, 3, 5));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(2, 3, 5));
}

TEST(LineTest, ShortestLineParallel) {
    Line line1({1, 1, 1}, {11, 1, 1});
    Line line2({1, 2, 1}, {11, 2, 1});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(1, 2, 1));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(1, 1, 1));
}

TEST(LineTest, ShortestLineEndToEndGap) {
    Line line1({2, 3, 7}, {10, 3, 7});
    Line line2({11, 4, 7}, {20, 4, 7});

    Line shortestLine = line1.shortestLineFrom(line2);

    EXPECT_EQ(shortestLine.getFrom(), Eigen::Vector3d(11, 4, 7));
    EXPECT_EQ(shortestLine.getTo(), Eigen::Vector3d(10, 3, 7));
}