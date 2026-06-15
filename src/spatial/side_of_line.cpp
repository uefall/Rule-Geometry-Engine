#include "rge/spatial/side_of_line.hpp"

#include <cmath>

namespace rge::spatial {
namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

Side side_of_line(Point2 point, Line line) noexcept {
    const double cross = (line.b - line.a).cross(point - line.a);
    if (std::abs(cross) <= kEpsilon) {
        return Side::kOn;
    }
    return cross > 0.0 ? Side::kLeft : Side::kRight;
}

Side side_of_line(BBox box, Line line, SidePolicy policy) noexcept {
    switch (policy) {
        case SidePolicy::kCenter:
            return side_of_line(box.center(), line);
        case SidePolicy::kAnyCorner: {
            Side result = Side::kOn;
            for (const Point2 corner : box.corners()) {
                const Side corner_side = side_of_line(corner, line);
                if (corner_side == Side::kOn) {
                    continue;
                }
                if (result == Side::kOn) {
                    result = corner_side;
                    continue;
                }
                if (result != corner_side) {
                    return Side::kOn;
                }
            }
            return result;
        }
        case SidePolicy::kAllCorners: {
            Side result = side_of_line(box.corners()[0], line);
            for (const Point2 corner : box.corners()) {
                if (side_of_line(corner, line) != result) {
                    return Side::kOn;
                }
            }
            return result;
        }
    }
    return Side::kOn;
}

bool crossed_line(Side previous, Side current,
                  CrossDirection allowed) noexcept {
    if (previous == current) {
        return false;
    }

    if (previous == Side::kOn || current == Side::kOn) {
        if (allowed == CrossDirection::kAny) {
            return true;
        }
        return false;
    }

    switch (allowed) {
        case CrossDirection::kAny:
            return true;
        case CrossDirection::kLeftToRight:
            return previous == Side::kLeft && current == Side::kRight;
        case CrossDirection::kRightToLeft:
            return previous == Side::kRight && current == Side::kLeft;
    }
    return false;
}

}  // namespace rge::spatial
