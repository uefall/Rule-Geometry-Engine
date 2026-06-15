#pragma once

#include <span>
#include <vector>

#include "rge/geom/line.hpp"
#include "rge/geom/polygon_view.hpp"
#include "rge/rules/events.hpp"
#include "rge/scratch_buffer.hpp"
#include "rge/spatial/side_of_line.hpp"
#include "rge/trajectory/track_state.hpp"

namespace rge::rules {

struct Region {
    std::size_t id{0};
    PolygonView polygon;
};

struct RuleLine {
    std::size_t id{0};
    Line segment;
    spatial::CrossDirection allowed_direction{spatial::CrossDirection::kAny};
};

struct RuleParams {
    double enter_overlap_threshold{0.5};
    double leave_overlap_threshold{0.3};
    std::size_t cross_line_min_frames{2};
    double cross_line_min_displacement{1.0};
    trajectory::Timestamp stay_min_duration{1000};
    double approach_min_speed{0.5};
    trajectory::FollowParams follow_params{};
};

template <std::size_t HistoryCapacity = 32>
struct TrackRuleState {
    std::vector<bool> inside_region;
    std::vector<trajectory::Timestamp> enter_timestamps;
    std::vector<std::optional<spatial::Side>> last_line_side;
    std::vector<std::size_t> stable_side_frames;
    std::vector<bool> stay_triggered;
};

template <std::size_t HistoryCapacity = 32>
struct RuleContext {
    std::span<const Region> regions;
    std::span<const RuleLine> lines;
    std::span<trajectory::TrackState<HistoryCapacity>> tracks;
    std::span<TrackRuleState<HistoryCapacity>> track_rules;
    RuleParams params;
    ScratchBuffer& scratch;
};

template <std::size_t HistoryCapacity = 32>
void initialize_rule_states(RuleContext<HistoryCapacity>& context);

template <std::size_t HistoryCapacity = 32>
std::vector<Event> evaluate_frame(RuleContext<HistoryCapacity>& context,
                                  const trajectory::TrackObservation& observation,
                                  const trajectory::TrackConfig& track_config);

template <std::size_t HistoryCapacity = 32>
std::vector<Event> evaluate_follow(RuleContext<HistoryCapacity>& context);

template <std::size_t HistoryCapacity = 32>
std::vector<Event> evaluate_approach(RuleContext<HistoryCapacity>& context,
                                     Point2 target);

}  // namespace rge::rules

#include "rge/rules/evaluators.inl"
