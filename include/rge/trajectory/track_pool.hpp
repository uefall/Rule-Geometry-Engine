#pragma once

#include <optional>
#include <span>
#include <vector>

#include "rge/trajectory/track_state.hpp"

namespace rge::trajectory {

template <std::size_t MaxTracks = 256, std::size_t HistoryCapacity = 32>
class TrackPool {
public:
    TrackState<HistoryCapacity>* find_or_create(uint64_t track_id) noexcept;

    void prune_inactive(Timestamp now, int64_t timeout_ms) noexcept;

    std::span<TrackState<HistoryCapacity>> active_tracks() noexcept;
    std::span<const TrackState<HistoryCapacity>> active_tracks() const noexcept;

    std::size_t size() const { return active_count_; }

private:
    std::array<TrackState<HistoryCapacity>, MaxTracks> tracks_{};
    std::array<bool, MaxTracks> active_{};
    std::array<Timestamp, MaxTracks> last_seen_{};
    std::size_t active_count_{0};
};

}  // namespace rge::trajectory

#include "rge/trajectory/track_pool.inl"
