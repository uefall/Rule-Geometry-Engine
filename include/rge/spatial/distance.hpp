#pragma once

#include "rge/geom/bbox.hpp"
#include "rge/geom/line.hpp"
#include "rge/geom/polygon_view.hpp"
#include "rge/geom/point.hpp"

namespace rge::spatial {

double distance(Point2 a, Point2 b) noexcept;
double distance(Point2 point, Line segment) noexcept;
double distance(Point2 point, PolygonView poly) noexcept;
double distance(BBox a, BBox b) noexcept;
double distance(BBox box, PolygonView poly) noexcept;

}  // namespace rge::spatial
