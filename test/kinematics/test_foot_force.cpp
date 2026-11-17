#include <gtest/gtest.h>

#include "test_common.hpp"
#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"

using gait_core::JointVec;
using gait_core::Scalar;
using gait_core::YawPitchPitchLeg;
using Vector3d = gait_core::Vec3;

static JointVec angles(double coxa, double femur, double tibia) {
    JointVec q(3);
    q << static_cast<Scalar>(coxa), static_cast<Scalar>(femur), static_cast<Scalar>(tibia);
    return q;
}

TEST(FootForce, RecoversAForceFromTheTorquesItCauses) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    JointVec q;
    ASSERT_TRUE(leg.ik(Vector3d(0.15, 0.02, -0.12), JointVec::Zero(3), q));
    const Vector3d force(3.0, -1.0, 7.5);  // N, ground on foot
    const JointVec torques = -leg.jacobian(q).transpose() * force;
    Vector3d estimated;
    ASSERT_TRUE(leg.groundForce(q, torques, estimated));
    EXPECT_NEAR((estimated - force).norm(), 0.0, tol(1e-6));
}

TEST(FootForce, JacobianMatchesAFiniteStep) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    const JointVec q = angles(0.2, -0.3, 0.4);
    const JointVec step = angles(0.001, -0.002, 0.0015);
    const Vector3d predicted = leg.fk(q) + leg.jacobian(q) * step;
    EXPECT_NEAR((leg.fk(q + step) - predicted).norm(), 0.0, tol(1e-5));
}

TEST(FootForce, StraightLegIsRejected) {
    const YawPitchPitchLeg leg{explorerRobot().legs[0]};
    // Femur and tibia in line: no vertical force can be found from the torques
    Vector3d force;
    EXPECT_FALSE(leg.groundForce(JointVec::Zero(3), angles(0.1, 0.2, 0.3), force));
}
