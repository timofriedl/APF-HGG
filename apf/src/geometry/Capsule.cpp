#include "Capsule.h"
#include <iostream>

std::pair<Line, double> Capsule::shortestLineFrom(const Capsule &other) const {
    assert(!std::isnan(line.getFrom().norm()));
    assert(!std::isnan(line.getTo().norm()));
    assert(!std::isnan(other.line.getFrom().norm()));
    assert(!std::isnan(other.line.getTo().norm()));

    Line centerToCenterLine = line.shortestLineFrom(other.line);

    Eigen::Vector3d dif = centerToCenterLine.getTo() - centerToCenterLine.getFrom();
    double norm = dif.norm();
    if (norm < std::numeric_limits<double>::epsilon())
        norm = std::numeric_limits<double>::max(); // If center lines cross, ignore collision

    Eigen::Vector3d normalized = dif / norm;

    Eigen::Vector3d newFrom = centerToCenterLine.getFrom() + other.radius * normalized;
    Eigen::Vector3d newTo = centerToCenterLine.getTo() - radius * normalized;
    Line newLine = {std::move(newFrom), std::move(newTo)};

    double newDistance = norm - radius - other.radius;
    return {newLine, newDistance};
}
