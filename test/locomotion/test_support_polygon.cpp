#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_common.hpp"
#include "gait_core/locomotion/support_polygon.hpp"

using gait_core::Scalar;
using gait_core::stabilityMargin;
using gait_core::Vec2;

namespace {

Vec2 v(double x, double y) {
    return Vec2(static_cast<Scalar>(x), static_cast<Scalar>(y));
}

Scalar margin(const std::vector<Vec2>& feet, const Vec2& point) {
    return stabilityMargin(feet.data(), static_cast<int>(feet.size()), point);
}

}  // namespace

TEST(SupportPolygon, InsideASquareIsTheDistanceToTheNearestEdge) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    EXPECT_NEAR(margin(square, v(0, 0)), 0.1, tol(1e-9));
    EXPECT_NEAR(margin(square, v(0.07, 0.02)), 0.03, tol(1e-9));
}

TEST(SupportPolygon, OutsideIsMinusTheDistanceToThePolygon) {
    const std::vector<Vec2> square{v(-0.1, -0.1), v(0.1, -0.1), v(0.1, 0.1), v(-0.1, 0.1)};
    EXPECT_NEAR(margin(square, v(0.15, 0)), -0.05, tol(1e-9));
    EXPECT_NEAR(margin(square, v(0.13, 0.14)), -0.05, tol(1e-9)) << "past a corner: distance to the corner";
    EXPECT_NEAR(margin(square, v(0.1, 0.05)), 0.0, tol(1e-9)) << "on an edge";
}

TEST(SupportPolygon, OrderRepeatsAndInnerFeetDoNotMatter) {
    const std::vector<Vec2> scrambled{v(0.1, 0.1), v(-0.1, -0.1), v(0, 0), v(-0.1, 0.1),
                                      v(0.1, -0.1), v(0.1, 0.1), v(0.05, 0.0)};
    EXPECT_NEAR(margin(scrambled, v(0, 0)), 0.1, tol(1e-9));
}

TEST(SupportPolygon, TwoFeetOrALineGiveNoArea) {
    // A trot's diagonal pair: the centre right on the line is only just balanced
    EXPECT_NEAR(margin({v(0.1, 0.1), v(-0.1, -0.1)}, v(0, 0)), 0.0, tol(1e-9));
    EXPECT_NEAR(margin({v(0.1, 0.1), v(-0.1, -0.1)}, v(0.1, -0.1)), -0.1 * std::sqrt(2.0), tol(1e-9));
    EXPECT_LE(margin({v(-0.1, 0), v(0, 0), v(0.1, 0)}, v(0, 0.01)), 0);
}

TEST(SupportPolygon, OneFootOrNone) {
    EXPECT_NEAR(margin({v(0.03, 0.04)}, v(0, 0)), -0.05, tol(1e-9));
    EXPECT_NEAR(margin({v(0.03, 0.04), v(0.03, 0.04)}, v(0, 0)), -0.05, tol(1e-9));
    EXPECT_TRUE(std::isinf(margin({}, v(0, 0))));
    EXPECT_LT(margin({}, v(0, 0)), 0);
}

TEST(SupportPolygon, ACrawlTriangleOfASquareRobotJustReachesItsCentre) {
    // Three corners of a square: the centre lies on the diagonal, so a crawl that lifts one leg at a
    // time without moving the body sideways is only just statically stable
    EXPECT_NEAR(margin({v(0.1, 0.1), v(0.1, -0.1), v(-0.1, -0.1)}, v(0, 0)), 0.0, tol(1e-9));
    EXPECT_GT(margin({v(0.1, 0.1), v(0.1, -0.1), v(-0.1, -0.1)}, v(0.02, -0.02)), 0);
}
