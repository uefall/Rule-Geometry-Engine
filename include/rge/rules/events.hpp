#pragma once

#include <cstdint>
#include <optional>

#include "rge/trajectory/track_state.hpp"

namespace rge::rules {

enum class EventKind {
    kEnterROI,
    kLeaveROI,
    kCrossLine,
    kStay,
    kFollow,
    kApproach,
};

struct Event {
    EventKind kind{EventKind::kEnterROI};
    uint64_t track_id{0};
    std::optional<std::size_t> region_id;
    std::optional<std::size_t> line_id;
    std::optional<uint64_t> target_track_id;
    trajectory::Timestamp timestamp{0};
};

}  // namespace rge::rules
