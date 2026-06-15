#pragma once

#include <cstdint>
#include <optional>

#include "rge/geom/bbox.hpp"
#include "rge/geom/point.hpp"
#include "rge/spatial/side_of_line.hpp"
#include "rge/trajectory/fixed_ring_buffer.hpp"

namespace rge::trajectory {

using Timestamp = int64_t;

struct TrackObservation {
    uint64_t track_id{0};
    Timestamp timestamp{0};
    BBox bbox{};
};

struct TrackConfig {
    enum class Anchor { kBBoxCenter, kBBoxBottomCenter };

    Anchor anchor{Anchor::kBBoxCenter};
    std::size_t velocity_window{3};
    double min_displacement{1.0};
};

template <std::size_t HistoryCapacity = 32>
struct TrackState {
    uint64_t track_id{0};
    FixedRingBuffer<Point2, HistoryCapacity> positions;
    FixedRingBuffer<Timestamp, HistoryCapacity> timestamps;
    FixedRingBuffer<BBox, HistoryCapacity> bboxes;

    std::optional<spatial::Side> last_side_of_line;
    Vec2 velocity{};
    double speed{0.0};
    double heading{0.0};
};

}  // namespace rge::trajectory
