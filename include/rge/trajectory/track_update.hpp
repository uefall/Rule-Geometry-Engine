#pragma once

#include "rge/error.hpp"
#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

Point2 anchor_point(BBox bbox, TrackConfig::Anchor anchor) noexcept;

template <std::size_t N>
Result<void> update(TrackState<N>& state, const TrackObservation& observation,
                    const TrackConfig& config) noexcept;

}  // namespace rge::trajectory

#include "rge/trajectory/track_update.inl"
