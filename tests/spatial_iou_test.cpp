#include <gtest/gtest.h>

#include "rge/rge.hpp"

TEST(SpatialIouTest, IdenticalBoxes) {
    const rge::BBox box = rge::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
    const auto result = rge::spatial::iou(box, box);
    ASSERT_TRUE(result.ok());
    EXPECT_NEAR(result.value(), 1.0, 1e-9);
}

TEST(SpatialIouTest, HalfOverlap) {
    const rge::BBox a = rge::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
    const rge::BBox b = rge::BBox::from_xywh(5.0, 0.0, 10.0, 10.0);
    const auto result = rge::spatial::iou(a, b);
    ASSERT_TRUE(result.ok());
    EXPECT_NEAR(result.value(), 1.0 / 3.0, 1e-9);
}

TEST(SpatialIouTest, BboxPolygonOverlapRatio) {
    rge::ScratchBuffer scratch;
    const rge::BBox box = rge::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
    const rge::Point2 square[] = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const rge::PolygonView poly{square};
    const auto ratio = rge::spatial::overlap_ratio(box, poly, scratch);
    ASSERT_TRUE(ratio.ok());
    EXPECT_NEAR(ratio.value(), 1.0, 1e-9);
}

TEST(SpatialIouTest, PolygonPolygonIou) {
    rge::ScratchBuffer scratch;
    const rge::Point2 a[] = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const rge::Point2 b[] = {{5.0, 0.0}, {15.0, 0.0}, {15.0, 10.0}, {5.0, 10.0}};
    const auto result = rge::spatial::iou(rge::PolygonView{a}, rge::PolygonView{b}, scratch);
    ASSERT_TRUE(result.ok());
    EXPECT_GT(result.value(), 0.0);
    EXPECT_LT(result.value(), 1.0);
}

TEST(SpatialContainsTest, PointInPolygon) {
    const rge::Point2 square[] = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const rge::PolygonView poly{square};
    EXPECT_TRUE(rge::spatial::contains(poly, {5.0, 5.0}));
    EXPECT_FALSE(rge::spatial::contains(poly, {15.0, 5.0}));
}
