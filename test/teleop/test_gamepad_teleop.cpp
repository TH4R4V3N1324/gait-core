// Checks the shared gamepad layout behaves like hexapod-ros2's controller_teleop did

#include <gtest/gtest.h>

#include "hexapod_protocol/packets.h"
#include "hexapod_teleop/gamepad_teleop.hpp"

using hexapod_teleop::Feature;
using hexapod_teleop::GamepadState;
using hexapod_teleop::GamepadTeleop;
using hexapod_teleop::RobotState;
using hexapod_teleop::TeleopOutput;

static_assert(static_cast<int>(Feature::TerrainAdaptation) == FEATURE_TERRAIN_ADAPTATION, "");
static_assert(static_cast<int>(Feature::WalkLeveling) == FEATURE_WALK_LEVELING, "");
static_assert(static_cast<int>(Feature::LoadBalancing) == FEATURE_LOAD_BALANCING, "");
static_assert(static_cast<int>(Feature::TiltLeveling) == FEATURE_TILT_LEVELING, "");
static_assert(hexapod_teleop::NUM_FEATURES == NUM_FEATURES, "");

namespace {

const RobotState STANDING{false, false};
const RobotState SEATED{true, false};
const RobotState TILT{false, true};

}  // namespace

TEST(GamepadTeleop, SticksFollowTheBodyFrame) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.leftX = 0.5f;    // right
    pad.leftY = 1.0f;    // forward
    pad.rightX = -0.25f; // left
    pad.rightY = 0.75f;  // forward
    pad.leftTrigger = 0.6f;
    pad.rightTrigger = 0.2f;
    const TeleopOutput out = teleop.update(pad, STANDING, 0.0);

    EXPECT_FLOAT_EQ(out.vx, 1.0f);
    EXPECT_FLOAT_EQ(out.vy, -0.5f);   // stick right is -y
    EXPECT_FLOAT_EQ(out.wz, 0.25f);   // stick left turns left
    EXPECT_FLOAT_EQ(out.shiftX, 1.0f);
    EXPECT_FLOAT_EQ(out.shiftY, -0.5f);
    EXPECT_FLOAT_EQ(out.roll, -0.25f);  // stick right lowers the right side, so left raises it
    EXPECT_FLOAT_EQ(out.pitch, 0.75f);  // stick forward lowers the nose
    EXPECT_FLOAT_EQ(out.yaw, 0.4f);     // left trigger turns left
}

TEST(GamepadTeleop, StartAndBackStepOncePerPress) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.start = true;
    EXPECT_TRUE(teleop.update(pad, STANDING, 0.0).nextMode);
    EXPECT_FALSE(teleop.update(pad, STANDING, 0.1).nextMode);  // still held
    pad.start = false;
    pad.back = true;
    const TeleopOutput out = teleop.update(pad, STANDING, 0.2);
    EXPECT_TRUE(out.nextGait);
    EXPECT_FALSE(out.nextMode);
}

TEST(GamepadTeleop, FaceButtonsToggleFeaturesInOrder) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.x = true;
    const TeleopOutput out = teleop.update(pad, STANDING, 0.0);
    EXPECT_FALSE(out.toggleFeature[0]);
    EXPECT_FALSE(out.toggleFeature[1]);
    EXPECT_TRUE(out.toggleFeature[static_cast<int>(Feature::LoadBalancing)]);
    EXPECT_FALSE(out.toggleFeature[3]);
}

TEST(GamepadTeleop, RightBumperSitsOrStandsWhenReleased) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.rightBumper = true;
    EXPECT_FALSE(teleop.update(pad, SEATED, 0.0).sit.has_value());  // nothing on the press
    pad.rightBumper = false;
    TeleopOutput out = teleop.update(pad, SEATED, 0.5);
    ASSERT_TRUE(out.sit.has_value());
    EXPECT_FALSE(*out.sit);  // seated, so stand up

    pad.rightBumper = true;
    teleop.update(pad, STANDING, 1.0);
    pad.rightBumper = false;
    out = teleop.update(pad, STANDING, 1.2);
    ASSERT_TRUE(out.sit.has_value());
    EXPECT_TRUE(*out.sit);
    EXPECT_FALSE(out.powerDown);
}

TEST(GamepadTeleop, HoldingTheRightBumperPowersDownInstead) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.rightBumper = true;
    teleop.update(pad, STANDING, 0.0);
    EXPECT_FALSE(teleop.update(pad, STANDING, 1.9).powerDown);
    EXPECT_TRUE(teleop.update(pad, STANDING, 2.0).powerDown);
    EXPECT_FALSE(teleop.update(pad, STANDING, 2.5).powerDown);  // once
    pad.rightBumper = false;
    EXPECT_FALSE(teleop.update(pad, STANDING, 3.0).sit.has_value());  // and no sit on release
}

TEST(GamepadTeleop, DpadStepsHeightAndRepeatsWhileHeld) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.dpadUp = true;
    EXPECT_EQ(teleop.update(pad, STANDING, 0.0).heightStep, 1);
    EXPECT_EQ(teleop.update(pad, STANDING, 0.1).heightStep, 0);
    EXPECT_EQ(teleop.update(pad, STANDING, 0.3).heightStep, 1);   // heightRepeat
    pad.dpadUp = false;
    pad.dpadDown = true;
    EXPECT_EQ(teleop.update(pad, STANDING, 0.35).heightStep, -1); // a new direction steps at once
}

TEST(GamepadTeleop, DpadLeavesHeightAloneWhenSeatedOrTilting) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.dpadUp = true;
    EXPECT_EQ(teleop.update(pad, SEATED, 0.0).heightStep, 0);
    const TeleopOutput out = teleop.update(pad, TILT, 1.0);
    EXPECT_EQ(out.heightStep, 0);
    EXPECT_FLOAT_EQ(out.shiftZ, 1.0f);  // in tilt mode it raises the body instead
}

TEST(GamepadTeleop, ResetForgetsHeldButtons) {
    GamepadTeleop teleop;
    GamepadState pad;
    pad.rightBumper = true;
    teleop.update(pad, STANDING, 0.0);
    teleop.reset();
    pad.rightBumper = false;
    EXPECT_FALSE(teleop.update(pad, STANDING, 0.5).sit.has_value());
}

TEST(StepHeight, AddsUpWhileMovingAndStaysInRange) {
    using hexapod_teleop::stepHeight;
    // No earlier target: from where the body is
    EXPECT_FLOAT_EQ(stepHeight(0.12f, std::nullopt, 1, 0.01f, 0.06f, 0.16f), 0.13f);
    // Still on its way to 0.13: from the target, so quick presses add up
    EXPECT_FLOAT_EQ(stepHeight(0.125f, 0.13f, 1, 0.01f, 0.06f, 0.16f), 0.14f);
    // Somewhere else entirely (it sat and stood): from where the body is
    EXPECT_FLOAT_EQ(stepHeight(0.08f, 0.13f, -1, 0.01f, 0.06f, 0.16f), 0.07f);
    // Clamped to the engine's range
    EXPECT_FLOAT_EQ(stepHeight(0.16f, std::nullopt, 1, 0.01f, 0.06f, 0.16f), 0.16f);
    EXPECT_FLOAT_EQ(stepHeight(0.065f, std::nullopt, -1, 0.01f, 0.06f, 0.16f), 0.06f);
}
