#pragma once

#include "rge/geom/line.hpp"
#include "rge/geom/point.hpp"
#include "rge/spatial/side_of_line.hpp"
#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

template <std::size_t N>
spatial::Side update_line_side(TrackState<N>& state, Line line,
                               spatial::SidePolicy policy) noexcept;

bool segment_intersects_line(Point2 p0, Point2 p1, Line line) noexcept;

}  // namespace rge::trajectory

#include "rge/trajectory/track_cross_line.inl"
