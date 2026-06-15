#pragma once

#include <cstddef>
#include <cstdint>

#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

struct FollowParams {
    double max_distance{50.0};
    double max_heading_diff{0.5};
    std::size_t min_frames{5};
};

struct PathSimParams {
    std::size_t compare_frames{16};
    double max_mean_distance{30.0};
};

template <std::size_t N>
double distance(const TrackState<N>& a, const TrackState<N>& b) noexcept;

template <std::size_t N>
double relative_bearing(const TrackState<N>& a, const TrackState<N>& b) noexcept;

template <std::size_t N>
double relative_speed(const TrackState<N>& a, const TrackState<N>& b) noexcept;

template <std::size_t N>
double heading_difference(const TrackState<N>& a, const TrackState<N>& b) noexcept;

template <std::size_t N>
bool is_following(const TrackState<N>& a, const TrackState<N>& b,
                  const FollowParams& params) noexcept;

template <std::size_t N>
double path_similarity(const TrackState<N>& a, const TrackState<N>& b,
                       const PathSimParams& params) noexcept;

template <std::size_t N>
double time_aligned_distance(const TrackState<N>& a, const TrackState<N>& b,
                             Timestamp max_dt) noexcept;

}  // namespace rge::trajectory

#include "rge/trajectory/track_relation.inl"
