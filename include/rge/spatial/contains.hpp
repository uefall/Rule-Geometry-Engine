#pragma once

#include "rge/geom/bbox.hpp"
#include "rge/geom/polygon_view.hpp"
#include "rge/geom/point.hpp"

namespace rge::spatial {

bool contains(BBox box, Point2 point, bool inclusive = true) noexcept;
bool contains(PolygonView poly, Point2 point) noexcept;
bool contains(BBox outer, BBox inner, bool inclusive = true) noexcept;
bool within(BBox box, PolygonView poly) noexcept;

}  // namespace rge::spatial
