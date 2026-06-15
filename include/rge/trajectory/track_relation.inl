#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "rge/spatial/distance.hpp"
#include "rge/trajectory/track_relation.hpp"

namespace rge::trajectory {

template <std::size_t N>
inline double distance(const TrackState<N>& a, const TrackState<N>& b) noexcept {
    if (a.positions.empty() || b.positions.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    return spatial::distance(a.positions.back(), b.positions.back());
}

template <std::size_t N>
inline double relative_bearing(const TrackState<N>& a,
                               const TrackState<N>& b) noexcept {
    if (a.positions.empty() || b.positions.empty()) {
        return 0.0;
    }
    const Point2 delta = b.positions.back() - a.positions.back();
    return std::atan2(delta.y, delta.x);
}

template <std::size_t N>
inline double relative_speed(const TrackState<N>& a,
                             const TrackState<N>& b) noexcept {
    return (a.velocity - b.velocity).length();
}

template <std::size_t N>
inline double heading_difference(const TrackState<N>& a,
                                 const TrackState<N>& b) noexcept {
    double diff = std::abs(a.heading - b.heading);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

inline double angle_between(double a, double b) {
    double diff = std::abs(a - b);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

template <std::size_t N>
inline bool is_following(const TrackState<N>& a, const TrackState<N>& b,
                         const FollowParams& params) noexcept {
    if (a.positions.size() < params.min_frames ||
        b.positions.size() < params.min_frames) {
        return false;
    }

    if (distance(a, b) > params.max_distance) {
        return false;
    }
    if (heading_difference(a, b) > params.max_heading_diff) {
        return false;
    }

    const double bearing = relative_bearing(a, b);
    return angle_between(a.heading, bearing) <= params.max_heading_diff;
}

template <std::size_t N>
inline double path_similarity(const TrackState<N>& a, const TrackState<N>& b,
                              const PathSimParams& params) noexcept {
    if (a.positions.empty() || b.positions.empty()) {
        return 0.0;
    }

    const std::size_t count =
        std::min({params.compare_frames, a.positions.size(), b.positions.size()});
    if (count == 0) {
        return 0.0;
    }

    double mean_distance = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t ai = a.positions.size() - count + i;
        const std::size_t bi = b.positions.size() - count + i;
        mean_distance += spatial::distance(a.positions[ai], b.positions[bi]);
    }
    mean_distance /= static_cast<double>(count);

    const double distance_score =
        1.0 - std::min(mean_distance / params.max_mean_distance, 1.0);
    const double heading_score =
        1.0 - std::min(heading_difference(a, b) / M_PI, 1.0);
    return std::clamp(0.5 * distance_score + 0.5 * heading_score, 0.0, 1.0);
}

template <std::size_t N>
inline double time_aligned_distance(const TrackState<N>& a, const TrackState<N>& b,
                                    Timestamp max_dt) noexcept {
    if (a.timestamps.empty() || b.timestamps.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    const Timestamp target = a.timestamps.back();
    std::size_t best_index = 0;
    Timestamp best_dt = std::numeric_limits<Timestamp>::max();
    for (std::size_t i = 0; i < b.timestamps.size(); ++i) {
        const Timestamp dt = std::abs(b.timestamps[i] - target);
        if (dt < best_dt) {
            best_dt = dt;
            best_index = i;
        }
    }

    if (best_dt > max_dt) {
        return std::numeric_limits<double>::infinity();
    }
    return spatial::distance(a.positions.back(), b.positions[best_index]);
}

}  // namespace rge::trajectory
