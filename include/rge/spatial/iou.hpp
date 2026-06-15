#pragma once

#include "rge/geom/bbox.hpp"
#include "rge/geom/line.hpp"
#include "rge/geom/polygon_view.hpp"
#include "rge/geom/point.hpp"
#include "rge/error.hpp"
#include "rge/scratch_buffer.hpp"

namespace rge::spatial {

Result<double> iou(BBox a, BBox b) noexcept;
Result<double> intersection_area(BBox a, BBox b) noexcept;
Result<double> iou(BBox box, PolygonView poly, ScratchBuffer& scratch) noexcept;
Result<double> iou(PolygonView a, PolygonView b, ScratchBuffer& scratch) noexcept;
Result<double> overlap_ratio(BBox box, PolygonView poly,
                             ScratchBuffer& scratch) noexcept;

}  // namespace rge::spatial
