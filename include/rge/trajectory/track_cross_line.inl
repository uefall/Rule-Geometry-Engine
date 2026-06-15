#pragma once

#include "rge/spatial/intersects.hpp"

namespace rge::trajectory {

template <std::size_t N>
inline spatial::Side update_line_side(TrackState<N>& state, Line line,
                                      spatial::SidePolicy policy) noexcept {
    if (state.positions.empty()) {
        return spatial::Side::kOn;
    }
    const spatial::Side current =
        spatial::side_of_line(state.positions.back(), line);
    state.last_side_of_line = current;
    return current;
}

inline bool segment_intersects_line(Point2 p0, Point2 p1, Line line) noexcept {
    return spatial::intersects(Line{p0, p1}, line);
}

}  // namespace rge::trajectory
