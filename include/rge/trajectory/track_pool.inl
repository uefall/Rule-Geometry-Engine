#pragma once

namespace rge::trajectory {

template <std::size_t MaxTracks, std::size_t HistoryCapacity>
inline TrackState<HistoryCapacity>* TrackPool<MaxTracks, HistoryCapacity>::find_or_create(
    uint64_t track_id) noexcept {
    for (std::size_t i = 0; i < MaxTracks; ++i) {
        if (active_[i] && tracks_[i].track_id == track_id) {
            last_seen_[i] = tracks_[i].timestamps.empty()
                                ? last_seen_[i]
                                : tracks_[i].timestamps.back();
            return &tracks_[i];
        }
    }

    for (std::size_t i = 0; i < MaxTracks; ++i) {
        if (!active_[i]) {
            tracks_[i] = TrackState<HistoryCapacity>{};
            tracks_[i].track_id = track_id;
            active_[i] = true;
            last_seen_[i] = 0;
            ++active_count_;
            return &tracks_[i];
        }
    }

    return nullptr;
}

template <std::size_t MaxTracks, std::size_t HistoryCapacity>
inline void TrackPool<MaxTracks, HistoryCapacity>::prune_inactive(
    Timestamp now, int64_t timeout_ms) noexcept {
    for (std::size_t i = 0; i < MaxTracks; ++i) {
        if (!active_[i]) {
            continue;
        }
        if (now - last_seen_[i] > timeout_ms) {
            active_[i] = false;
            if (active_count_ > 0) {
                --active_count_;
            }
        }
    }
}

template <std::size_t MaxTracks, std::size_t HistoryCapacity>
inline std::span<TrackState<HistoryCapacity>>
TrackPool<MaxTracks, HistoryCapacity>::active_tracks() noexcept {
    return std::span<TrackState<HistoryCapacity>>(tracks_.data(), MaxTracks);
}

template <std::size_t MaxTracks, std::size_t HistoryCapacity>
inline std::span<const TrackState<HistoryCapacity>>
TrackPool<MaxTracks, HistoryCapacity>::active_tracks() const noexcept {
    return std::span<const TrackState<HistoryCapacity>>(tracks_.data(), MaxTracks);
}

}  // namespace rge::trajectory
