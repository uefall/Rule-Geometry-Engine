#include "rge/spatial/contains.hpp"

#include <cmath>

namespace rge::spatial {
namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

bool contains(BBox box, Point2 point, bool inclusive) noexcept {
    return box.contains(point, inclusive);
}

bool contains(PolygonView poly, Point2 point) noexcept {
    if (poly.size() < 3) {
        return false;
    }

    bool inside = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Point2& pi = poly.vertices[i];
        const Point2& pj = poly.vertices[j];
        const bool intersects = ((pi.y > point.y) != (pj.y > point.y)) &&
                                (point.x < (pj.x - pi.x) * (point.y - pi.y) /
                                                   (pj.y - pi.y + kEpsilon) +
                                               pi.x);
        if (intersects) {
            inside = !inside;
        }
    }
    return inside;
}

bool contains(BBox outer, BBox inner, bool inclusive) noexcept {
    if (!outer.is_valid() || !inner.is_valid()) {
        return false;
    }
    for (const Point2 corner : inner.corners()) {
        if (!contains(outer, corner, inclusive)) {
            return false;
        }
    }
    return true;
}

bool within(BBox box, PolygonView poly) noexcept {
    if (!box.is_valid() || poly.size() < 3) {
        return false;
    }
    for (const Point2 corner : box.corners()) {
        if (!contains(poly, corner)) {
            return false;
        }
    }
    return true;
}

}  // namespace rge::spatial
