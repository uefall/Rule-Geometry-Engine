#pragma once

#include <span>

#include "rge/geom/point.hpp"
#include "rge/error.hpp"
#include "rge/scratch_buffer.hpp"

namespace rge::spatial {

double polygon_area(std::span<const Point2> vertices) noexcept;
Result<std::size_t> clip_polygon_by_rect(std::span<const Point2> subject,
                                           const Point2& rect_min,
                                           const Point2& rect_max,
                                           ScratchBuffer& scratch) noexcept;
Result<std::size_t> intersect_polygons(std::span<const Point2> subject,
                                       std::span<const Point2> clip,
                                       ScratchBuffer& scratch) noexcept;

}  // namespace rge::spatial
