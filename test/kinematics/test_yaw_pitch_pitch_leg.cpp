#include <gtest/gtest.h>

#include "test_common.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"

using gait_core::ChainKinematics;
using gait_core::JointVec;
using gait_core::Scalar;
using gait_core::YawPitchPitchLeg;
using Vector3d = gait_core::Vec3;

// The analytic solver against the general chain solver built from the same leg, which shares none
// of its maths

static JointVec angles(double coxa, double femur, double tibia) {
    JointVec q(3);
    q << static_cast<Scalar>(coxa), static_cast<Scalar>(femur), static_cast<Scalar>(tibia);
    return q;
}

TEST(YawPitchPitchLeg, HasThreeJoints) {
    EXPECT_EQ(YawPitchPitchLeg(explorerRobot().legs[0]).numJoints(), 3);
}

TEST(YawPitchPitchLeg, FkAndJacobianMatchTheChain) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const ChainKinematics chain{explorerRobot().legs[0]};
    for (const JointVec& q : {angles(0.2, -0.3, 0.4), angles(-1.1, 0.9, 2.2), angles(0.0, -1.5, -0.6)}) {
        EXPECT_NEAR((leg.fk(q) - chain.fk(q)).norm(), 0.0, tol(1e-12)) << q.transpose();
        EXPECT_NEAR((leg.jacobian(q) - chain.jacobian(q)).norm(), 0.0, tol(1e-12)) << q.transpose();
    }
}

TEST(YawPitchPitchLeg, IkRoundTrips) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const JointVec expected = angles(-0.21, -0.79, 1.65);
    JointVec solved;
    ASSERT_TRUE(leg.ik(leg.fk(expected), JointVec::Zero(3), solved));
    ASSERT_EQ(solved.size(), 3);
    EXPECT_NEAR((solved - expected).norm(), 0.0, tol(1e-9));
}

TEST(YawPitchPitchLeg, GroundForceMatchesTheChain) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const ChainKinematics chain{explorerRobot().legs[0]};
    JointVec q;
    ASSERT_TRUE(leg.ik(Vector3d(0.15, 0.02, -0.12), JointVec::Zero(3), q));
    const JointVec torques = angles(0.4, -0.9, 0.3);
    Vector3d expected;
    Vector3d force;
    ASSERT_TRUE(chain.groundForce(q, torques, expected));
    ASSERT_TRUE(leg.groundForce(q, torques, force));
    EXPECT_NEAR((force - expected).norm(), 0.0, tol(1e-6));
}

TEST(YawPitchPitchLeg, GroundForceRejectsAStraightLeg) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    Vector3d force;
    EXPECT_FALSE(leg.groundForce(JointVec::Zero(3), JointVec(Vector3d(0.1, 0.2, 0.3)), force));
}

TEST(YawPitchPitchLeg, IkReportsAnOutOfReachTarget) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const Vector3d farAway(1.0, 0.0, -0.1);
    JointVec q;
    EXPECT_FALSE(leg.ik(farAway, JointVec::Zero(3), q));
    // The nearest valid solution: the leg straight out, pointing at the target
    ASSERT_EQ(q.size(), 3);
    EXPECT_NEAR(q(2), 0.0, tol(1e-9)) << "knee straight";
    EXPECT_NEAR(leg.fk(q).norm(), 0.349, 0.01);
    EXPECT_LT(leg.fk(q).normalized().dot(farAway.normalized()), 1.0001);
    EXPECT_GT(leg.fk(q).normalized().dot(farAway.normalized()), 0.99);
}

TEST(YawPitchPitchLeg, IkReportsATargetTooCloseToReach) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    JointVec q;
    // Just past the coxa joint: closer to the femur joint than |femur - tibia|
    EXPECT_FALSE(leg.ik(Vector3d(0.06, 0.0, 0.0), JointVec::Zero(3), q));
}

TEST(YawPitchPitchLeg, TakesItsLinkLengthsFromTheLeg) {
    // A longer femur: the solver built from this leg must use it
    gait_core::LegDefinition definition = explorerRobot().legs[0];
    definition.joints[2].origin.x() = static_cast<Scalar>(0.12);
    const YawPitchPitchLeg leg{definition};
    EXPECT_NEAR((leg.fk(JointVec::Zero(3)) - Vector3d(static_cast<Scalar>(0.369), 0, 0)).norm(), 0.0, tol(1e-9));
}
