#include "Line.h"
#include "gte/DistSegmentSegment.h"
#include <iostream>

Line Line::shortestLineFrom(const Line &other) const {
    assert(!std::isnan(from.norm()));
    assert(!std::isnan(to.norm()));
    assert(!std::isnan(other.from.norm()));
    assert(!std::isnan(other.to.norm()));

    // Convert Eigen::Vector3d to gte::Vector<3, double>
    gte::Vector<3, double> a0{from.x(), from.y(), from.z()};
    gte::Vector<3, double> a1{to.x(), to.y(), to.z()};
    gte::Vector<3, double> b0{other.from.x(), other.from.y(), other.from.z()};
    gte::Vector<3, double> b1{other.to.x(), other.to.y(), other.to.z()};

    // Use the DCPQuery to compute the closest points
    gte::DCPSegment3Segment3<double> query;
    auto result = query({a0, a1}, {b0, b1});

    // Convert the closest points back to Eigen::Vector3d
    Eigen::Vector3d closestFrom(result.closest[1][0], result.closest[1][1], result.closest[1][2]);
    Eigen::Vector3d closestTo(result.closest[0][0], result.closest[0][1], result.closest[0][2]);

    return {std::move(closestFrom), std::move(closestTo)};
}
