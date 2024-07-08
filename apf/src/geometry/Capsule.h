#ifndef APF_HGG_CORE_CAPSULE_H
#define APF_HGG_CORE_CAPSULE_H

#include <Eigen/Dense>
#include "Line.h"


class Capsule {
    const Line line;
    const double radius;

public:
    Capsule(Line &&line, double radius)
            : line(std::move(line)), radius(radius) {}

    std::pair<Line, double> shortestLineFrom(const Capsule &other) const;

    const Line &getLine() const { return line; };
};


#endif //APF_HGG_CORE_CAPSULE_H
