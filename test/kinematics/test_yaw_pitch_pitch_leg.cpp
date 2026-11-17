#include <gtest/gtest.h>

#include "test_common.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"
#include "gait_core/kinematics/kinematic_solver.hpp"

using gait_core::YawPitchPitchLeg;
using gait_core::JointAngles;
using gait_core::JointVec;
using gait_core::KinematicSolver;
using Vector3d = gait_core::Vec3;

// The adapter must give exactly what KinematicSolver gives: it only converts the joint types

static JointVec vec(const JointAngles& a) {
    JointVec v(3);
    v << a.coxa, a.femur, a.tibia;
    return v;
}

TEST(YawPitchPitchLeg, HasThreeJoints) {
    EXPECT_EQ(YawPitchPitchLeg(explorerRobot().legs[0]).numJoints(), 3);
}

TEST(YawPitchPitchLeg, IkMatchesKinematicSolver) {
    const KinematicSolver solver{explorerRobot().legs[0]};
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const Vector3d foot(0.24896, -0.05286, -0.07071);
    const JointAngles expected = solver.solveIK(foot);
    JointVec angles;
    EXPECT_TRUE(leg.ik(foot, JointVec::Zero(3), angles));
    ASSERT_EQ(angles.size(), 3);
    EXPECT_EQ(angles(0), expected.coxa);
    EXPECT_EQ(angles(1), expected.femur);
    EXPECT_EQ(angles(2), expected.tibia);
}

TEST(YawPitchPitchLeg, FkAndJacobianMatchKinematicSolver) {
    const KinematicSolver solver{explorerRobot().legs[0]};
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const JointAngles angles{0.2, -0.3, 0.4};
    EXPECT_EQ(leg.fk(vec(angles)), solver.solveFK(angles));
    EXPECT_EQ(leg.jacobian(vec(angles)), solver.jacobian(angles));
}

TEST(YawPitchPitchLeg, GroundForceMatchesKinematicSolver) {
    const KinematicSolver solver{explorerRobot().legs[0]};
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const JointAngles angles = solver.solveIK(Vector3d(0.15, 0.02, -0.12));
    const Vector3d torques(0.4, -0.9, 0.3);
    Vector3d expected;
    Vector3d force;
    ASSERT_TRUE(solver.groundForce(angles, torques, expected));
    ASSERT_TRUE(leg.groundForce(vec(angles), JointVec(torques), force));
    EXPECT_NEAR((force - expected).norm(), 0.0, tol(1e-6));
}

TEST(YawPitchPitchLeg, GroundForceRejectsAStraightLeg) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    Vector3d force;
    EXPECT_FALSE(leg.groundForce(JointVec::Zero(3), JointVec(Vector3d(0.1, 0.2, 0.3)), force));
}

TEST(YawPitchPitchLeg, IkReportsAnOutOfReachTarget) {
    const KinematicSolver solver{explorerRobot().legs[0]};
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const Vector3d farAway(1.0, 0.0, -0.1);
    JointVec angles;
    EXPECT_FALSE(leg.ik(farAway, JointVec::Zero(3), angles));
    // Still the clamped solution solveIK has always given
    const JointAngles expected = solver.solveIK(farAway);
    EXPECT_EQ(angles(0), expected.coxa);
    EXPECT_EQ(angles(1), expected.femur);
    EXPECT_EQ(angles(2), expected.tibia);
}

TEST(YawPitchPitchLeg, IkReportsATargetTooCloseToReach) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    JointVec angles;
    // Just past the coxa joint: closer to the femur joint than |femur - tibia|
    EXPECT_FALSE(leg.ik(Vector3d(0.06, 0.0, 0.0), JointVec::Zero(3), angles));
}
