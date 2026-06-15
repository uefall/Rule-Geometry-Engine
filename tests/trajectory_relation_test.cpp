#include <gtest/gtest.h>

#include "rge/rge.hpp"

namespace {

void feed_parallel_track(rge::trajectory::TrackState<16>& track, uint64_t id,
                         double y, int frames) {
    rge::trajectory::TrackConfig config{};
    for (int i = 0; i < frames; ++i) {
        ASSERT_TRUE(rge::trajectory::update(
                        track,
                        {id, static_cast<rge::trajectory::Timestamp>(i),
                         rge::BBox::from_xywh(static_cast<double>(i * 2), y, 2.0, 2.0)},
                        config)
                        .ok());
    }
}

}  // namespace

TEST(TrajectoryRelationTest, HeadingDifferenceSmallForParallelTracks) {
    rge::trajectory::TrackState<16> a{};
    rge::trajectory::TrackState<16> b{};
    feed_parallel_track(a, 1, 0.0, 8);
    feed_parallel_track(b, 2, 5.0, 8);

    EXPECT_NEAR(rge::trajectory::heading_difference(a, b), 0.0, 1e-6);
    EXPECT_LT(rge::trajectory::distance(a, b), 10.0);
}

TEST(TrajectoryRelationTest, IsFollowingDetectsTailgating) {
    rge::trajectory::TrackState<16> leader{};
    rge::trajectory::TrackState<16> follower{};
    rge::trajectory::TrackConfig config{};

    for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(rge::trajectory::update(
                        leader,
                        {1, static_cast<rge::trajectory::Timestamp>(i),
                         rge::BBox::from_xywh(static_cast<double>(i * 2), 0.0, 2.0, 2.0)},
                        config)
                        .ok());
        ASSERT_TRUE(rge::trajectory::update(
                        follower,
                        {2, static_cast<rge::trajectory::Timestamp>(i),
                         rge::BBox::from_xywh(static_cast<double>(i * 2 - 4), 0.0, 2.0, 2.0)},
                        config)
                        .ok());
    }

    rge::trajectory::FollowParams params{};
    params.max_distance = 10.0;
    params.max_heading_diff = 0.2;
    params.min_frames = 5;

    EXPECT_TRUE(rge::trajectory::is_following(follower, leader, params));
}

TEST(TrajectoryRelationTest, PathSimilarityIsHighForParallelTracks) {
    rge::trajectory::TrackState<16> a{};
    rge::trajectory::TrackState<16> b{};
    feed_parallel_track(a, 1, 0.0, 12);
    feed_parallel_track(b, 2, 4.0, 12);

    rge::trajectory::PathSimParams params{};
    params.compare_frames = 8;
    params.max_mean_distance = 10.0;

    EXPECT_GT(rge::trajectory::path_similarity(a, b, params), 0.5);
}

TEST(TrajectoryRelationTest, TrackPoolFindOrCreate) {
    rge::trajectory::TrackPool<8, 16> pool{};
    auto* first = pool.find_or_create(42);
    auto* second = pool.find_or_create(42);
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first, second);
}
