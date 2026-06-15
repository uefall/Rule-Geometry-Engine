#include <gtest/gtest.h>

#include "rge/rge.hpp"

TEST(RoiEnterLeaveTest, EnterLeaveWithHysteresis) {
    rge::ScratchBuffer scratch;
    const rge::Point2 square[] = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};

    std::array<rge::rules::Region, 1> regions{
        rge::rules::Region{0, rge::PolygonView{square}},
    };
    std::array<rge::rules::RuleLine, 0> lines{};
    std::array<rge::trajectory::TrackState<16>, 1> tracks{};
    std::array<rge::rules::TrackRuleState<16>, 1> track_rules{};

    rge::rules::RuleContext<16> context{
        regions,
        lines,
        std::span<rge::trajectory::TrackState<16>>(tracks),
        std::span<rge::rules::TrackRuleState<16>>(track_rules),
        {},
        scratch,
    };
    rge::rules::initialize_rule_states(context);

    rge::trajectory::TrackConfig track_config{};
    auto events = rge::rules::evaluate_frame(
        context, {1, 0, rge::BBox::from_xywh(15.0, 15.0, 2.0, 2.0)}, track_config);
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].kind, rge::rules::EventKind::kEnterROI);

    events = rge::rules::evaluate_frame(
        context, {1, 100, rge::BBox::from_xywh(25.0, 25.0, 2.0, 2.0)}, track_config);
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].kind, rge::rules::EventKind::kLeaveROI);
}

TEST(RoiEnterLeaveTest, StayEventAfterDuration) {
    rge::ScratchBuffer scratch;
    const rge::Point2 square[] = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};

    std::array<rge::rules::Region, 1> regions{
        rge::rules::Region{0, rge::PolygonView{square}},
    };
    std::array<rge::rules::RuleLine, 0> lines{};
    std::array<rge::trajectory::TrackState<16>, 1> tracks{};
    std::array<rge::rules::TrackRuleState<16>, 1> track_rules{};

    rge::rules::RuleParams params{};
    params.stay_min_duration = 50;

    rge::rules::RuleContext<16> context{
        regions,
        lines,
        std::span<rge::trajectory::TrackState<16>>(tracks),
        std::span<rge::rules::TrackRuleState<16>>(track_rules),
        params,
        scratch,
    };
    rge::rules::initialize_rule_states(context);

    rge::trajectory::TrackConfig track_config{};
    rge::rules::evaluate_frame(context,
                               {1, 0, rge::BBox::from_xywh(1.0, 1.0, 2.0, 2.0)},
                               track_config);
    const auto events = rge::rules::evaluate_frame(
        context, {1, 100, rge::BBox::from_xywh(1.0, 1.0, 2.0, 2.0)}, track_config);

    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].kind, rge::rules::EventKind::kStay);
}

TEST(RoiEnterLeaveTest, ApproachAndFollowEvents) {
    rge::ScratchBuffer scratch;
    std::array<rge::rules::Region, 0> regions{};
    std::array<rge::rules::RuleLine, 0> lines{};
    std::array<rge::trajectory::TrackState<16>, 2> tracks{};
    std::array<rge::rules::TrackRuleState<16>, 2> track_rules{};

    rge::rules::RuleParams params{};
    params.approach_min_speed = 0.01;
    params.follow_params.max_distance = 8.0;
    params.follow_params.max_heading_diff = 0.3;
    params.follow_params.min_frames = 5;

    rge::rules::RuleContext<16> context{
        regions,
        lines,
        std::span<rge::trajectory::TrackState<16>>(tracks),
        std::span<rge::rules::TrackRuleState<16>>(track_rules),
        params,
        scratch,
    };
    rge::rules::initialize_rule_states(context);

    rge::trajectory::TrackConfig config{};
    for (int i = 10; i >= 1; --i) {
        rge::rules::evaluate_frame(
            context,
            {1, static_cast<rge::trajectory::Timestamp>(10 - i),
             rge::BBox::from_xywh(static_cast<double>(i), 0.0, 2.0, 2.0)},
            config);
    }

    const auto approach =
        rge::rules::evaluate_approach(context, rge::Point2{0.0, 0.0});
    EXPECT_FALSE(approach.empty());

    for (int i = 0; i < 10; ++i) {
        rge::rules::evaluate_frame(
            context,
            {1, static_cast<rge::trajectory::Timestamp>(100 + i),
             rge::BBox::from_xywh(static_cast<double>(i * 2), 0.0, 2.0, 2.0)},
            config);
        rge::rules::evaluate_frame(
            context,
            {2, static_cast<rge::trajectory::Timestamp>(100 + i),
             rge::BBox::from_xywh(static_cast<double>(i * 2 - 4), 0.0, 2.0, 2.0)},
            config);
    }

    const auto follow = rge::rules::evaluate_follow(context);
    EXPECT_FALSE(follow.empty());
}
