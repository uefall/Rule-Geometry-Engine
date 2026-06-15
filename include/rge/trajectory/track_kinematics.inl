#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "rge/trajectory/track_kinematics.hpp"
namespace {

inline double normalize_angle(double angle) {
    while (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    while (angle >= 2.0 * M_PI) {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

inline double angle_difference(double a, double b) {
    double diff = std::abs(a - b);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

}  // namespace

namespace rge::trajectory {

template <std::size_t N>
inline Vec2 velocity(const TrackState<N>& state, std::size_t window) noexcept {
    if (state.positions.size() < 2) {
        return {};
    }
    const std::size_t effective_window =
        std::min(window, state.positions.size() - 1);
    return state.positions.back() -
           state.positions[state.positions.size() - 1 - effective_window];
}

template <std::size_t N>
inline double speed(const TrackState<N>& state) noexcept {
    return state.speed;
}

template <std::size_t N>
inline double heading(const TrackState<N>& state) noexcept {
    return state.heading;
}

template <std::size_t N>
inline double displacement(const TrackState<N>& state, std::size_t frames) noexcept {
    if (state.positions.size() < 2 || frames == 0) {
        return 0.0;
    }
    const std::size_t effective_frames =
        std::min(frames, state.positions.size() - 1);
    return (state.positions.back() -
            state.positions[state.positions.size() - 1 - effective_frames])
        .length();
}

template <std::size_t N>
inline bool is_stationary(const TrackState<N>& state,
                          const TrackConfig& config) noexcept {
    return state.speed <= config.min_displacement &&
           displacement(state, 1) <= config.min_displacement;
}

template <std::size_t N>
inline Point2 smooth_position(const TrackState<N>& state,
                              std::size_t window) noexcept {
    if (state.positions.empty()) {
        return {};
    }
    const std::size_t count = std::min(window, state.positions.size());
    Point2 sum{};
    for (std::size_t i = state.positions.size() - count; i < state.positions.size(); ++i) {
        sum = sum + state.positions[i];
    }
    return sum * (1.0 / static_cast<double>(count));
}

template <std::size_t N>
inline Point2 predict_position(const TrackState<N>& state,
                               Timestamp timestamp) noexcept {
    if (state.positions.empty()) {
        return {};
    }
    const Timestamp dt = timestamp - state.timestamps.back();
    return state.positions.back() + state.velocity * static_cast<double>(dt);
}

template <std::size_t N>
inline double path_length(const TrackState<N>& state) noexcept {
    if (state.positions.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < state.positions.size(); ++i) {
        length += (state.positions[i] - state.positions[i - 1]).length();
    }
    return length;
}

template <std::size_t N>
inline double turning_angle(const TrackState<N>& state) noexcept {
    if (state.positions.size() < 3) {
        return 0.0;
    }
    const Point2 p0 = state.positions[state.positions.size() - 3];
    const Point2 p1 = state.positions[state.positions.size() - 2];
    const Point2 p2 = state.positions.back();
    const Point2 v1 = p1 - p0;
    const Point2 v2 = p2 - p1;
    const double denom = v1.length() * v2.length();
    if (denom <= 0.0) {
        return 0.0;
    }
    const double cosine = std::clamp(v1.dot(v2) / denom, -1.0, 1.0);
    return std::acos(cosine);
}

template <std::size_t N>
inline Result<DistanceTrend> distance_trend(const TrackState<N>& state, Point2 target,
                                            std::size_t window,
                                            double stable_epsilon) noexcept {
    if (state.positions.size() < 2) {
        return Result<DistanceTrend>::err(GeomError::kInsufficientHistory);
    }

    const std::size_t count = std::min(window, state.positions.size());
    if (count < 2) {
        return Result<DistanceTrend>::err(GeomError::kInsufficientHistory);
    }

    double first = (state.positions[state.positions.size() - count] - target).length();
    double last = (state.positions.back() - target).length();
    const double delta = last - first;

    if (std::abs(delta) <= stable_epsilon) {
        return Result<DistanceTrend>::ok(DistanceTrend::kStable);
    }
    if (delta < 0.0) {
        return Result<DistanceTrend>::ok(DistanceTrend::kApproaching);
    }
    return Result<DistanceTrend>::ok(DistanceTrend::kReceding);
}

}  // namespace rge::trajectory
