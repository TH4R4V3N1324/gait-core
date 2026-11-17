#include <gtest/gtest.h>

#include <cmath>

#include "test_common.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"

using gait_core::JointVec;
using gait_core::YawPitchPitchLeg;
using Vector3d = gait_core::Vec3;

// Poses measured from the Hexapod_V6.1 CAD (Hexapod_V6.1_stand.step and _walk.step): the foot
// is the point 0.186 m from the knee axis along the line through the foot ball's centre,
// in the leg frame (+x out from the coxa axis, z from the hip plane)

static constexpr double DEG = M_PI / 180.0;

TEST(Kinematics, IkMatchesTheCadStandPose) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const Vector3d foot(0.17965, 0.0, -0.13353);
    JointVec angles;
    ASSERT_TRUE(leg.ik(foot, JointVec::Zero(3), angles));
    EXPECT_NEAR(angles(0), 0.0, tol(1e-6));
    EXPECT_NEAR(angles(1), -30.02 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles(2), 110.71 * DEG, 0.1 * DEG);
    EXPECT_NEAR((leg.fk(angles) - foot).norm(), 0.0, tol(1e-9));
}

TEST(Kinematics, IkMatchesTheCadWalkPose) {
    // A swinging leg: coxa turned 12 degrees, femur raised 45 degrees, foot lifted
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const Vector3d foot(0.24896, -0.05286, -0.07071);
    JointVec angles;
    ASSERT_TRUE(leg.ik(foot, JointVec::Zero(3), angles));
    EXPECT_NEAR(angles(0), -12.0 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles(1), -45.01 * DEG, 0.1 * DEG);
    EXPECT_NEAR(angles(2), 94.50 * DEG, 0.1 * DEG);
    EXPECT_NEAR((leg.fk(angles) - foot).norm(), 0.0, tol(1e-9));
}
