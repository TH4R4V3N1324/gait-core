#include <gtest/gtest.h>

#include <cmath>

#include "test_common.hpp"
#include "gait_core/gait_config.hpp"
#include "gait_core/trajectory_generator.hpp"

using gait_core::BodyPose;
using gait_core::GaitConfig;
using gait_core::NUM_LEGS;
using gait_core::TrajectoryGenerator;
using Vector3d = gait_core::Vec3;

// Legs run clockwise from the front right: the first three on the right, the last three on the left

/**
 * @brief Each foot's position (leg frame) after the body takes up a pose over the start positions
 * @param[in] pose The body pose
 * @return The foot position of each leg, indexed by Leg
*/
static std::array<Vector3d, NUM_LEGS> feetFor(const BodyPose& pose) {
    TrajectoryGenerator gen;
    GaitConfig config;
    std::array<Vector3d, NUM_LEGS> feet;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        feet[leg] = gen.posedFoot(leg, config.startPosition.at(leg), pose);
    }
    return feet;
}

TEST(BodyPose, NeutralPoseLeavesTheFeetWhereTheyAre) {
    GaitConfig config;
    auto feet = feetFor(BodyPose{});
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_NEAR((feet[leg] - config.startPosition.at(leg)).norm(), 0.0, tol(1e-12)) << "leg " << leg;
    }
}

TEST(BodyPose, RaisingTheBodyLowersEveryFoot) {
    GaitConfig config;
    BodyPose pose;
    pose.translation = Vector3d(0.0, 0.0, 0.02);
    auto feet = feetFor(pose);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_NEAR(feet[leg].z(), config.startPosition.at(leg).z() - 0.02, tol(1e-12)) << "leg " << leg;
    }
}

TEST(BodyPose, ShiftingForwardMovesTheFeetBackInTheBodyFrame) {
    TrajectoryGenerator gen;
    GaitConfig config;
    BodyPose pose;
    pose.translation = Vector3d(0.03, 0.0, 0.0);
    auto feet = feetFor(pose);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Vector3d moved = gen.legToBody(leg, feet[leg]) - gen.legToBody(leg, config.startPosition.at(leg));
        EXPECT_NEAR((moved - Vector3d(-0.03, 0.0, 0.0)).norm(), 0.0, tol(1e-12)) << "leg " << leg;
    }
}

TEST(BodyPose, PositiveRollLowersTheRightSide) {
    GaitConfig config;
    BodyPose pose;
    pose.roll = 0.1;
    auto feet = feetFor(pose);
    // The right hips drop towards the ground, so the right feet rise relative to them
    for (int leg : {gait_core::LEG_FRONT_RIGHT, gait_core::LEG_MIDDLE_RIGHT, gait_core::LEG_REAR_RIGHT}) {
        EXPECT_GT(feet[leg].z(), config.startPosition.at(leg).z() + 0.01) << "leg " << leg;
    }
    for (int leg : {gait_core::LEG_REAR_LEFT, gait_core::LEG_MIDDLE_LEFT, gait_core::LEG_FRONT_LEFT}) {
        EXPECT_LT(feet[leg].z(), config.startPosition.at(leg).z() - 0.01) << "leg " << leg;
    }
}

TEST(BodyPose, PositivePitchLowersTheNose) {
    GaitConfig config;
    BodyPose pose;
    pose.pitch = 0.1;
    auto feet = feetFor(pose);
    for (int leg : {gait_core::LEG_FRONT_RIGHT, gait_core::LEG_FRONT_LEFT}) {
        EXPECT_GT(feet[leg].z(), config.startPosition.at(leg).z() + 0.01) << "leg " << leg;
    }
    for (int leg : {gait_core::LEG_REAR_RIGHT, gait_core::LEG_REAR_LEFT}) {
        EXPECT_LT(feet[leg].z(), config.startPosition.at(leg).z() - 0.01) << "leg " << leg;
    }
}

TEST(BodyPose, PositiveYawSweepsTheFeetClockwiseAroundTheBody) {
    TrajectoryGenerator gen;
    GaitConfig config;
    BodyPose pose;
    pose.yaw = 0.1;
    auto feet = feetFor(pose);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Vector3d before = gen.legToBody(leg, config.startPosition.at(leg));
        Vector3d after = gen.legToBody(leg, feet[leg]);
        double turned = std::atan2(after.y(), after.x()) - std::atan2(before.y(), before.x());
        EXPECT_NEAR(std::remainder(turned, 2.0 * M_PI), -0.1, tol(1e-9)) << "leg " << leg;
        EXPECT_NEAR(after.head<2>().norm(), before.head<2>().norm(), tol(1e-12)) << "leg " << leg;
    }
}

TEST(BodyPose, RotationAboutAPointLeavesThatPointInPlace) {
    TrajectoryGenerator gen;
    const Vector3d pivot(0.0, 0.0, -0.12);
    const BodyPose pose = TrajectoryGenerator::rotationAbout(0.1, -0.08, pivot);
    EXPECT_NEAR((gen.unposedBodyPoint(pivot, pose) - pivot).norm(), 0.0, tol(1e-12));
}

TEST(BodyPose, UnposingUndoesPosing) {
    TrajectoryGenerator gen;
    GaitConfig config;
    BodyPose pose = TrajectoryGenerator::rotationAbout(0.12, 0.07, Vector3d(0.0, 0.0, -0.12));
    pose.yaw = 0.05;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Vector3d foot = config.startPosition.at(leg);
        const Vector3d posed = gen.legToBody(leg, gen.posedFoot(leg, foot, pose));
        EXPECT_NEAR((gen.unposedBodyPoint(posed, pose) - gen.legToBody(leg, foot)).norm(), 0.0, tol(1e-12)) << "leg " << leg;
    }
}

TEST(BodyPose, LevelingAboutTheGroundBarelyMovesTheFeetSideways) {
    // Walk leveling's largest correction, about the ground under the body: the feet change height
    // but move less than 9 mm sideways (test_leg_clearance covers stride circles 9 mm wider)
    TrajectoryGenerator gen;
    GaitConfig config;
    const BodyPose pose = TrajectoryGenerator::rotationAbout(0.15, 0.15, Vector3d(0.0, 0.0, config.currentHeight));
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const Vector3d offset = gen.posedFoot(leg, config.startPosition.at(leg), pose) - config.startPosition.at(leg);
        EXPECT_LT(offset.head<2>().norm(), 0.009) << "leg " << leg;
    }
}
