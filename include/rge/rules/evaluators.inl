#pragma once

#include "rge/spatial/iou.hpp"
#include "rge/spatial/side_of_line.hpp"
#include "rge/trajectory/track_cross_line.hpp"
#include "rge/trajectory/track_kinematics.hpp"
#include "rge/trajectory/track_relation.hpp"
#include "rge/trajectory/track_update.hpp"

#include <algorithm>

namespace rge::rules {
namespace {

template <std::size_t HistoryCapacity>
trajectory::TrackState<HistoryCapacity>* find_track(
    std::span<trajectory::TrackState<HistoryCapacity>> tracks, uint64_t track_id) {
    for (auto& track : tracks) {
        if (track.track_id == track_id) {
            return &track;
        }
    }
    return nullptr;
}

template <std::size_t HistoryCapacity>
TrackRuleState<HistoryCapacity>* find_rule_state(
    std::span<TrackRuleState<HistoryCapacity>> states, uint64_t track_id,
    std::span<trajectory::TrackState<HistoryCapacity>> tracks) {
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        if (tracks[i].track_id == track_id) {
            return i < states.size() ? &states[i] : nullptr;
        }
    }
    return nullptr;
}

}  // namespace

template <std::size_t HistoryCapacity>
inline void initialize_rule_states(RuleContext<HistoryCapacity>& context) {
    for (auto& state : context.track_rules) {
        state.inside_region.assign(context.regions.size(), false);
        state.enter_timestamps.assign(context.regions.size(), 0);
        state.last_line_side.assign(context.lines.size(), std::nullopt);
        state.stable_side_frames.assign(context.lines.size(), 0);
        state.stay_triggered.assign(context.regions.size(), false);
    }
}

template <std::size_t HistoryCapacity>
inline std::vector<Event> evaluate_frame(
    RuleContext<HistoryCapacity>& context,
    const trajectory::TrackObservation& observation,
    const trajectory::TrackConfig& track_config) {
    std::vector<Event> events;
    if (context.tracks.empty() || context.track_rules.empty()) {
        return events;
    }

    auto* track = find_track(context.tracks, observation.track_id);
    if (track == nullptr) {
        for (auto& slot : context.tracks) {
            if (slot.track_id == 0) {
                track = &slot;
                break;
            }
        }
        if (track == nullptr) {
            track = &context.tracks.front();
        }
    }

    trajectory::update(*track, observation, track_config);
    TrackRuleState<HistoryCapacity>* rule_state =
        find_rule_state(context.track_rules, observation.track_id, context.tracks);
    if (rule_state == nullptr) {
        rule_state = &context.track_rules[0];
    }

    for (std::size_t region_index = 0; region_index < context.regions.size();
         ++region_index) {
        const Region& region = context.regions[region_index];
        const Result<double> overlap = spatial::overlap_ratio(
            observation.bbox, region.polygon, context.scratch);
        const double ratio = overlap.ok() ? overlap.value() : 0.0;
        const bool inside_now = ratio >= context.params.enter_overlap_threshold;
        const bool inside_before = rule_state->inside_region[region_index];

        if (!inside_before && inside_now) {
            events.push_back(Event{EventKind::kEnterROI, observation.track_id,
                                   region_index, std::nullopt, std::nullopt,
                                   observation.timestamp});
            rule_state->enter_timestamps[region_index] = observation.timestamp;
            rule_state->stay_triggered[region_index] = false;
        } else if (inside_before &&
                   ratio < context.params.leave_overlap_threshold) {
            events.push_back(Event{EventKind::kLeaveROI, observation.track_id,
                                   region_index, std::nullopt, std::nullopt,
                                   observation.timestamp});
        } else if (inside_before && !rule_state->stay_triggered[region_index] &&
                   observation.timestamp - rule_state->enter_timestamps[region_index] >=
                       context.params.stay_min_duration) {
            events.push_back(Event{EventKind::kStay, observation.track_id, region_index,
                                   std::nullopt, std::nullopt, observation.timestamp});
            rule_state->stay_triggered[region_index] = true;
        }

        rule_state->inside_region[region_index] =
            ratio >= context.params.leave_overlap_threshold;
    }

    if (track->positions.size() >= 2) {
        const Point2 previous = track->positions[track->positions.size() - 2];
        const Point2 current = track->positions.back();
        for (std::size_t line_index = 0; line_index < context.lines.size(); ++line_index) {
            const RuleLine& rule_line = context.lines[line_index];
            const spatial::Side previous_side = spatial::side_of_line(previous, rule_line.segment);
            const spatial::Side current_side = spatial::side_of_line(current, rule_line.segment);
            const bool segment_hit =
                trajectory::segment_intersects_line(previous, current, rule_line.segment);

            if (segment_hit &&
                spatial::crossed_line(previous_side, current_side,
                                      rule_line.allowed_direction)) {
                events.push_back(Event{EventKind::kCrossLine, observation.track_id,
                                       std::nullopt, line_index, std::nullopt,
                                       observation.timestamp});
            }
        }
    }

    return events;
}

template <std::size_t HistoryCapacity>
inline std::vector<Event> evaluate_follow(RuleContext<HistoryCapacity>& context) {
    std::vector<Event> events;
    for (std::size_t i = 0; i < context.tracks.size(); ++i) {
        for (std::size_t j = i + 1; j < context.tracks.size(); ++j) {
            const auto& a = context.tracks[i];
            const auto& b = context.tracks[j];
            if (a.track_id == 0 || b.track_id == 0) {
                continue;
            }
            if (trajectory::is_following(a, b, context.params.follow_params)) {
                events.push_back(Event{EventKind::kFollow, a.track_id, std::nullopt,
                                       std::nullopt, b.track_id,
                                       b.timestamps.empty() ? 0 : b.timestamps.back()});
            } else if (trajectory::is_following(b, a, context.params.follow_params)) {
                events.push_back(Event{EventKind::kFollow, b.track_id, std::nullopt,
                                       std::nullopt, a.track_id,
                                       a.timestamps.empty() ? 0 : a.timestamps.back()});
            }
        }
    }
    return events;
}

template <std::size_t HistoryCapacity>
inline std::vector<Event> evaluate_approach(RuleContext<HistoryCapacity>& context,
                                            Point2 target) {
    std::vector<Event> events;
    for (const auto& track : context.tracks) {
        if (track.track_id == 0 || track.positions.empty()) {
            continue;
        }
        const Result<trajectory::DistanceTrend> trend =
            trajectory::distance_trend(track, target);
        if (trend.ok() && trend.value() == trajectory::DistanceTrend::kApproaching &&
            track.speed >= context.params.approach_min_speed) {
            events.push_back(Event{EventKind::kApproach, track.track_id, std::nullopt,
                                   std::nullopt, std::nullopt,
                                   track.timestamps.empty() ? 0 : track.timestamps.back()});
        }
    }
    return events;
}

}  // namespace rge::rules
