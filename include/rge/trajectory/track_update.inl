#pragma once

#include <algorithm>
#include <cmath>

#include "rge/geom/bbox.hpp"
#include "rge/geom/point.hpp"
#include "rge/error.hpp"
#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

inline Point2 anchor_point(BBox bbox, TrackConfig::Anchor anchor) noexcept {
    switch (anchor) {
        case TrackConfig::Anchor::kBBoxBottomCenter:
            return bbox.bottom_center();
        case TrackConfig::Anchor::kBBoxCenter:
        default:
            return bbox.center();
    }
}

template <std::size_t N>
inline Result<void> update(TrackState<N>& state,
                           const TrackObservation& observation,
                           const TrackConfig& config) noexcept {
    if (!observation.bbox.is_valid()) {
        return Result<void>::err(GeomError::kDegenerateInput);
    }

    state.track_id = observation.track_id;
    state.positions.push(anchor_point(observation.bbox, config.anchor));
    state.timestamps.push(observation.timestamp);
    state.bboxes.push(observation.bbox);

    if (state.positions.size() >= 2) {
        const std::size_t window = std::min(config.velocity_window, state.positions.size() - 1);
        const Point2 delta =
            state.positions.back() - state.positions[state.positions.size() - 1 - window];
        const Timestamp dt =
            state.timestamps.back() - state.timestamps[state.timestamps.size() - 1 - window];
        if (dt > 0) {
            const double inv_dt = 1.0 / static_cast<double>(dt);
            state.velocity = delta * inv_dt;
            state.speed = state.velocity.length();
            state.heading = std::atan2(state.velocity.y, state.velocity.x);
            if (state.heading < 0.0) {
                state.heading += 2.0 * M_PI;
            }
        }
    }

    return Result<void>::success();
}

}  // namespace rge::trajectory
