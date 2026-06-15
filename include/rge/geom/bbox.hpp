#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "rge/geom/point.hpp"

namespace rge {

struct BBox {
    Point2 min{};
    Point2 max{};

    static BBox from_xywh(double x, double y, double width, double height) {
        return BBox{{x, y}, {x + width, y + height}};
    }

    static BBox from_points(Point2 a, Point2 b) {
        return BBox{
            {std::min(a.x, b.x), std::min(a.y, b.y)},
            {std::max(a.x, b.x), std::max(a.y, b.y)},
        };
    }

    bool is_valid() const { return min.x <= max.x && min.y <= max.y; }

    double width() const { return max.x - min.x; }
    double height() const { return max.y - min.y; }

    double area() const {
        if (!is_valid()) {
            return 0.0;
        }
        return width() * height();
    }

    double perimeter() const {
        if (!is_valid()) {
            return 0.0;
        }
        return 2.0 * (width() + height());
    }

    Point2 center() const {
        return {(min.x + max.x) * 0.5, (min.y + max.y) * 0.5};
    }

    Point2 bottom_center() const { return {(min.x + max.x) * 0.5, max.y}; }

    bool contains(const Point2& point, bool inclusive = true) const {
        if (!is_valid()) {
            return false;
        }
        if (inclusive) {
            return point.x >= min.x && point.x <= max.x && point.y >= min.y &&
                   point.y <= max.y;
        }
        return point.x > min.x && point.x < max.x && point.y > min.y &&
               point.y < max.y;
    }

    BBox expand(double margin) const {
        return BBox{{min.x - margin, min.y - margin},
                      {max.x + margin, max.y + margin}};
    }

    BBox clamp_to(const BBox& bounds) const {
        return BBox{
            {std::max(min.x, bounds.min.x), std::max(min.y, bounds.min.y)},
            {std::min(max.x, bounds.max.x), std::min(max.y, bounds.max.y)},
        };
    }

    BBox union_with(const BBox& other) const {
        return BBox{
            {std::min(min.x, other.min.x), std::min(min.y, other.min.y)},
            {std::max(max.x, other.max.x), std::max(max.y, other.max.y)},
        };
    }

    BBox intersect_with(const BBox& other) const {
        return BBox{
            {std::max(min.x, other.min.x), std::max(min.y, other.min.y)},
            {std::min(max.x, other.max.x), std::min(max.y, other.max.y)},
        };
    }

    std::array<Point2, 4> corners() const {
        return {
            Point2{min.x, min.y},
            Point2{max.x, min.y},
            Point2{max.x, max.y},
            Point2{min.x, max.y},
        };
    }
};

}  // namespace rge
