#pragma once

#include "rge/geom/bbox.hpp"
#include "rge/geom/line.hpp"
#include "rge/geom/point.hpp"

namespace rge::spatial {

enum class Side { kLeft, kRight, kOn };

enum class SidePolicy { kCenter, kAnyCorner, kAllCorners };

enum class CrossDirection { kAny, kLeftToRight, kRightToLeft };

Side side_of_line(Point2 point, Line line) noexcept;
Side side_of_line(BBox box, Line line, SidePolicy policy) noexcept;

bool crossed_line(Side previous, Side current,
                  CrossDirection allowed = CrossDirection::kAny) noexcept;

}  // namespace rge::spatial
