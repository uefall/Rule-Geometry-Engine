#include <gtest/gtest.h>

#include "rge/rge.hpp"

TEST(SideOfLineTest, LeftRightAndOn) {
    const rge::Line line{{0.0, 0.0}, {10.0, 0.0}};
    EXPECT_EQ(rge::spatial::side_of_line({5.0, 5.0}, line), rge::spatial::Side::kLeft);
    EXPECT_EQ(rge::spatial::side_of_line({5.0, -5.0}, line), rge::spatial::Side::kRight);
    EXPECT_EQ(rge::spatial::side_of_line({5.0, 0.0}, line), rge::spatial::Side::kOn);
}

TEST(SideOfLineTest, CrossedLineDirection) {
    using rge::spatial::CrossDirection;
    using rge::spatial::Side;
    EXPECT_TRUE(rge::spatial::crossed_line(Side::kLeft, Side::kRight, CrossDirection::kAny));
    EXPECT_TRUE(rge::spatial::crossed_line(Side::kLeft, Side::kRight,
                                           CrossDirection::kLeftToRight));
    EXPECT_FALSE(rge::spatial::crossed_line(Side::kRight, Side::kLeft,
                                            CrossDirection::kLeftToRight));
}
