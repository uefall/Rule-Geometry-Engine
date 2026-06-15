#include <gtest/gtest.h>

#include "rge/rge.hpp"

TEST(TrajectoryUpdateTest, ComputesVelocityAndHeading) {
    rge::trajectory::TrackState<8> track{};
    rge::trajectory::TrackConfig config{};
    config.velocity_window = 1;

    ASSERT_TRUE(rge::trajectory::update(
                    track, {1, 0, rge::BBox::from_xywh(0.0, 0.0, 2.0, 2.0)}, config)
                    .ok());
    ASSERT_TRUE(rge::trajectory::update(
                    track, {1, 10, rge::BBox::from_xywh(10.0, 0.0, 2.0, 2.0)}, config)
                    .ok());

    EXPECT_NEAR(rge::trajectory::speed(track), 1.0, 1e-9);
    EXPECT_NEAR(rge::trajectory::heading(track), 0.0, 1e-9);
    EXPECT_NEAR(rge::trajectory::displacement(track, 1), 10.0, 1e-9);
}

TEST(TrajectoryUpdateTest, DistanceTrendApproaching) {
    rge::trajectory::TrackState<8> track{};
    rge::trajectory::TrackConfig config{};
    const rge::Point2 target{0.0, 0.0};

    for (int i = 10; i >= 1; --i) {
        ASSERT_TRUE(rge::trajectory::update(
                        track,
                        {1, static_cast<rge::trajectory::Timestamp>(10 - i),
                         rge::BBox::from_xywh(static_cast<double>(i), 0.0, 1.0, 1.0)},
                        config)
                        .ok());
    }

    const auto trend = rge::trajectory::distance_trend(track, target);
    ASSERT_TRUE(trend.ok());
    EXPECT_EQ(trend.value(), rge::trajectory::DistanceTrend::kApproaching);
}

TEST(TrajectoryUpdateTest, StationaryTrack) {
    rge::trajectory::TrackState<8> track{};
    rge::trajectory::TrackConfig config{};
    config.min_displacement = 0.5;

    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(rge::trajectory::update(
                        track, {1, static_cast<rge::trajectory::Timestamp>(i),
                                rge::BBox::from_xywh(0.0, 0.0, 2.0, 2.0)},
                        config)
                        .ok());
    }

    EXPECT_TRUE(rge::trajectory::is_stationary(track, config));
}
