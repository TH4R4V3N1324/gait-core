#include <gtest/gtest.h>

#include "test_common.hpp"
#include "gait_core/kinematics/kinematic_solver.hpp"

using gait_core::JointAngles;
using gait_core::KinematicSolver;
using Vector3d = gait_core::Vec3;

TEST(FootForce, RecoversAForceFromTheTorquesItCauses) {
    KinematicSolver solver{explorerRobot().legs[0]};
    JointAngles angles = solver.solveIK(Vector3d(0.15, 0.02, -0.12));
    const Vector3d force(3.0, -1.0, 7.5);  // N, ground on foot
    const Vector3d torques = -solver.jacobian(angles).transpose() * force;
    Vector3d estimated;
    ASSERT_TRUE(solver.groundForce(angles, torques, estimated));
    EXPECT_NEAR((estimated - force).norm(), 0.0, tol(1e-6));
}

TEST(FootForce, JacobianMatchesAFiniteStep) {
    KinematicSolver solver{explorerRobot().legs[0]};
    JointAngles angles{0.2, -0.3, 0.4};
    const Vector3d step(0.001, -0.002, 0.0015);
    JointAngles moved{angles.coxa + step.x(), angles.femur + step.y(), angles.tibia + step.z()};
    const Vector3d predicted = solver.solveFK(angles) + solver.jacobian(angles) * step;
    EXPECT_NEAR((solver.solveFK(moved) - predicted).norm(), 0.0, tol(1e-5));
}

TEST(FootForce, StraightLegIsRejected) {
    KinematicSolver solver{explorerRobot().legs[0]};
    // Femur and tibia in line: no vertical force can be found from the torques
    JointAngles straight{0.0, 0.0, 0.0};
    Vector3d force;
    EXPECT_FALSE(solver.groundForce(straight, Vector3d(0.1, 0.2, 0.3), force));
}
