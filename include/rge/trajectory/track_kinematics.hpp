#pragma once

#include <cmath>

#include "rge/error.hpp"
#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

enum class DistanceTrend { kApproaching, kReceding, kStable };

template <std::size_t N>
Vec2 velocity(const TrackState<N>& state, std::size_t window) noexcept;

template <std::size_t N>
double speed(const TrackState<N>& state) noexcept;

template <std::size_t N>
double heading(const TrackState<N>& state) noexcept;

template <std::size_t N>
double displacement(const TrackState<N>& state, std::size_t frames) noexcept;

template <std::size_t N>
bool is_stationary(const TrackState<N>& state, const TrackConfig& config) noexcept;

template <std::size_t N>
Point2 smooth_position(const TrackState<N>& state, std::size_t window) noexcept;

template <std::size_t N>
Point2 predict_position(const TrackState<N>& state, Timestamp timestamp) noexcept;

template <std::size_t N>
double path_length(const TrackState<N>& state) noexcept;

template <std::size_t N>
double turning_angle(const TrackState<N>& state) noexcept;

template <std::size_t N>
Result<DistanceTrend> distance_trend(const TrackState<N>& state, Point2 target,
                                     std::size_t window = 5,
                                     double stable_epsilon = 2.0) noexcept;

}  // namespace rge::trajectory

#include "rge/trajectory/track_kinematics.inl"
