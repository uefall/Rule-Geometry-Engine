#pragma once

#include "rge/geom/point.hpp"

namespace rge {

struct Line {
    Point2 a{};
    Point2 b{};

    double length() const { return (b - a).length(); }

    Point2 direction() const {
        const double len = length();
        if (len <= 0.0) {
            return {};
        }
        return (b - a) * (1.0 / len);
    }

    Point2 normal() const {
        const Point2 dir = direction();
        return {-dir.y, dir.x};
    }
};

}  // namespace rge
