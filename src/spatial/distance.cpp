#include "rge/spatial/distance.hpp"

#include "rge/spatial/contains.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rge::spatial {
namespace {

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

double distance_point_segment(Point2 point, Point2 a, Point2 b) {
    const Point2 ab = b - a;
    const double length_squared = ab.length_squared();
    if (length_squared <= 0.0) {
        return (point - a).length();
    }
    const double t = clamp((point - a).dot(ab) / length_squared, 0.0, 1.0);
    const Point2 projection = a + ab * t;
    return (point - projection).length();
}

double distance_point_polygon_boundary(Point2 point, PolygonView poly) {
    if (poly.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }

    double min_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const Point2 a = poly.vertices[i];
        const Point2 b = poly.vertices[(i + 1) % poly.size()];
        min_distance = std::min(min_distance, distance_point_segment(point, a, b));
    }
    return min_distance;
}

}  // namespace

double distance(Point2 a, Point2 b) noexcept { return (a - b).length(); }

double distance(Point2 point, Line segment) noexcept {
    return distance_point_segment(point, segment.a, segment.b);
}

double distance(Point2 point, PolygonView poly) noexcept {
    if (poly.size() < 3) {
        return std::numeric_limits<double>::infinity();
    }
    if (contains(poly, point)) {
        return 0.0;
    }
    return distance_point_polygon_boundary(point, poly);
}

double distance(BBox a, BBox b) noexcept {
    const double dx = std::max({0.0, a.min.x - b.max.x, b.min.x - a.max.x});
    const double dy = std::max({0.0, a.min.y - b.max.y, b.min.y - a.max.y});
    return std::hypot(dx, dy);
}

double distance(BBox box, PolygonView poly) noexcept {
    return distance(box.center(), poly);
}

}  // namespace rge::spatial
