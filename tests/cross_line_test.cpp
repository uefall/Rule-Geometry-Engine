#include <gtest/gtest.h>

#include "rge/rge.hpp"

TEST(CrossLineTest, DetectsHorizontalCrossing) {
    rge::trajectory::TrackState<16> track{};
    rge::trajectory::TrackConfig config{};
    const rge::Line line{{5.0, 0.0}, {5.0, 10.0}};

    bool crossed = false;

    for (int x = 0; x <= 10; ++x) {
        ASSERT_TRUE(rge::trajectory::update(
                        track,
                        {1, static_cast<rge::trajectory::Timestamp>(x),
                         rge::BBox::from_xywh(static_cast<double>(x), 4.0, 2.0, 2.0)},
                        config)
                        .ok());

        if (track.positions.size() < 2) {
            continue;
        }

        const rge::Point2 prev = track.positions[track.positions.size() - 2];
        const rge::Point2 curr = track.positions.back();
        const auto prev_side = rge::spatial::side_of_line(prev, line);
        const auto curr_side = rge::spatial::side_of_line(curr, line);

        if (rge::trajectory::segment_intersects_line(prev, curr, line) &&
            rge::spatial::crossed_line(prev_side, curr_side)) {
            crossed = true;
            break;
        }
    }

    EXPECT_TRUE(crossed);
}
