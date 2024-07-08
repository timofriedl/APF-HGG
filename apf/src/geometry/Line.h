#ifndef APF_HGG_CORE_LINE_H
#define APF_HGG_CORE_LINE_H


#include <Eigen/Dense>
#include <utility>

class Line {
    const Eigen::Vector3d from;
    const Eigen::Vector3d to;

public:
    Line(Eigen::Vector3d &&from, Eigen::Vector3d &&to)
            : from(std::move(from)), to(std::move(to)) {}

    Line shortestLineFrom(const Line &other) const;

    const Eigen::Vector3d &getFrom() const { return from; }

    const Eigen::Vector3d &getTo() const { return to; }
};


#endif //APF_HGG_CORE_LINE_H
