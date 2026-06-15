#pragma once

#include "rge/geom/bbox.hpp"
#include "rge/geom/line.hpp"
#include "rge/geom/polygon_view.hpp"
#include "rge/geom/point.hpp"
#include "rge/scratch_buffer.hpp"

namespace rge::spatial {

bool intersects(BBox a, BBox b) noexcept;
bool intersects(BBox box, Line segment) noexcept;
bool intersects(Line a, Line b) noexcept;
bool intersects(BBox box, PolygonView poly, ScratchBuffer& scratch) noexcept;
bool intersects(PolygonView a, PolygonView b, ScratchBuffer& scratch) noexcept;

}  // namespace rge::spatial
