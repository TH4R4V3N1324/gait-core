#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "test_common.hpp"
#include "gait_core/estimation/leg_odometry.hpp"

using gait_core::BodyMotion;
using gait_core::LegOdometry;
using Vector2d = gait_core::Vec2;

// Three planted feet of a tripod, in the body frame (+x forward, +y left)
static const std::vector<Vector2d> TRIPOD = {{0.17, -0.19}, {-0.17, -0.19}, {0.0, 0.24}};

/**
 * @brief Where world-fixed feet appear in the body frame after the body moves
 * @param[in] feet The feet in the body frame before the move
 * @param[in] motion The body's move, in its frame before the move
 * @return The same feet in the body frame after the move
*/
static std::vector<Vector2d> afterBodyMove(const std::vector<Vector2d>& feet, const BodyMotion& motion) {
    gait_core::Rotation2D inverse(-motion.dyaw);
    std::vector<Vector2d> moved;
    for (const auto& foot : feet) {
        moved.push_back(inverse * (foot - Vector2d(motion.dx, motion.dy)));
    }
    return moved;
}

/**
 * @brief Check estimateMotion recovers a known body motion from the feet
 * @param[in] truth The body motion to apply
*/
static void expectRecovers(const BodyMotion& truth) {
    BodyMotion estimate;
    ASSERT_TRUE(LegOdometry::estimateMotion(TRIPOD, afterBodyMove(TRIPOD, truth), estimate));
    EXPECT_NEAR(estimate.dx, truth.dx, tol(1e-9));
    EXPECT_NEAR(estimate.dy, truth.dy, tol(1e-9));
    EXPECT_NEAR(estimate.dyaw, truth.dyaw, tol(1e-9));
}

TEST(LegOdometry, RecoversForward) {
    expectRecovers({0.002, 0.0, 0.0});
}

TEST(LegOdometry, RecoversLeft) {
    expectRecovers({0.0, 0.002, 0.0});
}

TEST(LegOdometry, RecoversTurnLeft) {
    expectRecovers({0.0, 0.0, 0.01});
}

TEST(LegOdometry, RecoversCombinedMotion) {
    expectRecovers({0.003, -0.001, -0.02});
}

TEST(LegOdometry, NeedsTwoFeet) {
    BodyMotion estimate;
    EXPECT_FALSE(LegOdometry::estimateMotion({{0.1, 0.1}}, {{0.1, 0.1}}, estimate));
    EXPECT_FALSE(LegOdometry::estimateMotion(TRIPOD, {{0.1, 0.1}}, estimate));
}

TEST(LegOdometry, IntegratesInTheOdomFrame) {
    LegOdometry odom;
    // Turn left 90 degrees in steps, then drive forward 1 m: should end up at (0, 1)
    for (int i = 0; i < 90; ++i) {
        odom.integrate({0.0, 0.0, M_PI / 180.0});
    }
    for (int i = 0; i < 100; ++i) {
        odom.integrate({0.01, 0.0, 0.0});
    }
    EXPECT_NEAR(odom.x(), 0.0, tol(1e-9));
    EXPECT_NEAR(odom.y(), 1.0, tol(1e-9));
    EXPECT_NEAR(odom.yaw(), M_PI / 2.0, tol(1e-9));
}

TEST(LegOdometry, ClimbIsHowFarThePlantedFeetDrop) {
    // A tripod standing 0.134 m below the hips; the body rises 5 mm, so every foot reads 5 mm lower
    gait_core::Scalar climb;
    ASSERT_TRUE(LegOdometry::estimateClimb({-0.134, -0.134, -0.134}, {-0.139, -0.139, -0.139}, climb));
    EXPECT_NEAR(climb, 0.005, tol(1e-12));
}

TEST(LegOdometry, ClimbAveragesTheFeet) {
    // One foot sinking into soft ground only counts for its share
    gait_core::Scalar climb;
    ASSERT_TRUE(LegOdometry::estimateClimb({-0.10, -0.12, -0.14}, {-0.10, -0.12, -0.17}, climb));
    EXPECT_NEAR(climb, 0.01, tol(1e-12));
}

TEST(LegOdometry, ClimbNeedsTwoFeet) {
    gait_core::Scalar climb;
    EXPECT_FALSE(LegOdometry::estimateClimb({-0.134}, {-0.139}, climb));
    EXPECT_FALSE(LegOdometry::estimateClimb({-0.134, -0.134}, {-0.139}, climb));
}

TEST(LegOdometry, IntegratesClimbAndResets) {
    LegOdometry odom;
    // Up one 6 cm stair in steps
    for (int i = 0; i < 60; ++i) {
        odom.integrateClimb(0.001);
    }
    EXPECT_NEAR(odom.z(), 0.06, tol(1e-9));
    odom.reset();
    EXPECT_EQ(odom.z(), 0.0);
}
