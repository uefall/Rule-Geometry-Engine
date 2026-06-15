#include "rge/spatial/intersects.hpp"

#include "rge/spatial/contains.hpp"
#include "rge/spatial/polygon_ops.hpp"

#include <algorithm>

namespace rge::spatial {
namespace {

constexpr double kEpsilon = 1e-9;

double orientation(Point2 a, Point2 b, Point2 c) {
    return (b - a).cross(c - a);
}

bool on_segment(Point2 a, Point2 b, Point2 p) {
    return std::min(a.x, b.x) - kEpsilon <= p.x &&
           p.x <= std::max(a.x, b.x) + kEpsilon &&
           std::min(a.y, b.y) - kEpsilon <= p.y &&
           p.y <= std::max(a.y, b.y) + kEpsilon;
}

bool segments_intersect(Point2 p1, Point2 p2, Point2 q1, Point2 q2) {
    const double o1 = orientation(p1, p2, q1);
    const double o2 = orientation(p1, p2, q2);
    const double o3 = orientation(q1, q2, p1);
    const double o4 = orientation(q1, q2, p2);

    if (((o1 > kEpsilon && o2 < -kEpsilon) || (o1 < -kEpsilon && o2 > kEpsilon)) &&
        ((o3 > kEpsilon && o4 < -kEpsilon) || (o3 < -kEpsilon && o4 > kEpsilon))) {
        return true;
    }

    if (std::abs(o1) <= kEpsilon && on_segment(p1, p2, q1)) {
        return true;
    }
    if (std::abs(o2) <= kEpsilon && on_segment(p1, p2, q2)) {
        return true;
    }
    if (std::abs(o3) <= kEpsilon && on_segment(q1, q2, p1)) {
        return true;
    }
    if (std::abs(o4) <= kEpsilon && on_segment(q1, q2, p2)) {
        return true;
    }
    return false;
}

bool segment_intersects_polygon(Point2 a, Point2 b, PolygonView poly) {
    if (poly.size() < 3) {
        return false;
    }
    if (contains(poly, a) || contains(poly, b)) {
        return true;
    }
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Point2 c = poly.vertices[i];
        const Point2 d = poly.vertices[(i + 1) % poly.size()];
        if (segments_intersect(a, b, c, d)) {
            return true;
        }
    }
    return false;
}

bool bbox_intersects_polygon(BBox box, PolygonView poly) {
    if (!box.is_valid() || poly.size() < 3) {
        return false;
    }
    for (const Point2 corner : box.corners()) {
        if (contains(poly, corner)) {
            return true;
        }
    }
    for (const Point2 vertex : poly.vertices) {
        if (contains(box, vertex)) {
            return true;
        }
    }
    const auto corners = box.corners();
    for (std::size_t i = 0; i < corners.size(); ++i) {
        const Point2 a = corners[i];
        const Point2 b = corners[(i + 1) % corners.size()];
        if (segment_intersects_polygon(a, b, poly)) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool intersects(BBox a, BBox b) noexcept {
    if (!a.is_valid() || !b.is_valid()) {
        return false;
    }
    return !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y ||
             a.min.y > b.max.y);
}

bool intersects(BBox box, Line segment) noexcept {
    if (!box.is_valid()) {
        return false;
    }
    if (contains(box, segment.a) || contains(box, segment.b)) {
        return true;
    }
    const auto corners = box.corners();
    for (std::size_t i = 0; i < corners.size(); ++i) {
        if (segments_intersect(segment.a, segment.b, corners[i],
                               corners[(i + 1) % corners.size()])) {
            return true;
        }
    }
    return false;
}

bool intersects(Line a, Line b) noexcept {
    return segments_intersect(a.a, a.b, b.a, b.b);
}

bool intersects(BBox box, PolygonView poly, ScratchBuffer&) noexcept {
    return bbox_intersects_polygon(box, poly);
}

bool intersects(PolygonView a, PolygonView b, ScratchBuffer& scratch) noexcept {
    if (a.size() < 3 || b.size() < 3) {
        return false;
    }
    if (bbox_intersects_polygon(a.bounding_box(), b) == false &&
        bbox_intersects_polygon(b.bounding_box(), a) == false) {
        return false;
    }
    for (const Point2 vertex : a.vertices) {
        if (contains(b, vertex)) {
            return true;
        }
    }
    for (const Point2 vertex : b.vertices) {
        if (contains(a, vertex)) {
            return true;
        }
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Point2 p1 = a.vertices[i];
        const Point2 p2 = a.vertices[(i + 1) % a.size()];
        if (segment_intersects_polygon(p1, p2, b)) {
            return true;
        }
    }
    const Result<std::size_t> clipped =
        intersect_polygons(a.vertices, b.vertices, scratch);
    return clipped.ok() && clipped.value() >= 3;
}

}  // namespace rge::spatial
