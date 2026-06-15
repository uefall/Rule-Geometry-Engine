#pragma once

#include <span>

#include "rge/geom/bbox.hpp"
#include "rge/geom/point.hpp"

namespace rge {

struct PolygonView {
    std::span<const Point2> vertices;

    bool empty() const { return vertices.empty(); }
    std::size_t size() const { return vertices.size(); }

    double signed_area() const;
    BBox bounding_box() const;
};

struct PolylineView {
    std::span<const Point2> points;

    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }
};

BBox polygon_bounding_box(std::span<const Point2> vertices);
double polygon_signed_area(std::span<const Point2> vertices);

}  // namespace rge
