#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "test_common.hpp"
#include "gait_core/gait_engine.hpp"

using gait_core::CommandResult;
using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::GaitParams;
using gait_core::ImuSample;
using gait_core::JointFeedback;
using gait_core::JointVec;
using gait_core::explorer::NUM_LEGS;
using gait_core::Scalar;
using gait_core::SitState;
using gait_core::Timestamp;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);  // s
constexpr Timestamp DT_NS = 10'000'000;

/**
 * @brief Runs an engine with perfect servos: every command comes straight back as the joint state
 * @note The servos only answer while the engine wants them powered and the e-stop is released,
 *       like the leg hubs' SERVO_EN and kill loop
*/
class Robot {
public:
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    Timestamp now = 1'000'000'000;
    JointFeedback feedback;
    int commandedTicks = 0;
    bool estop = false;
    bool servoPower;   // what the engine last asked for

    explicit Robot(const GaitParams& params = GaitParams{}) : engine(params, explorerRobot()), servoPower(params.power_on_startup) {
        // Legs laid out flat at their zeros
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = JointVec::Zero(3);
        }
    }

    /**
     * @brief The robot slumps onto its belly with its legs at their zeros
    */
    void collapse() {
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.angles[leg] = JointVec::Zero(3);
        }
    }

    void run(double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        for (int i = 0; i < ticks; ++i) {
            if (servoPower && !estop) {
                feedback.stamp = now;
                engine.onJointStates(feedback);
            }
            now += DT_NS;
            const GaitOutput out = engine.update(DT, now);
            servoPower = out.servoPower;
            if (out.commanded) {
                ++commandedTicks;
                for (int leg = 0; leg < NUM_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }
};

}  // namespace

TEST(GaitEngine, WaitsForEveryJointBeforeMoving) {
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    JointFeedback partial;
    for (int leg = 0; leg < gait_core::explorer::LEG_FRONT_LEFT; ++leg) {  // the front left leg missing
        partial.present[leg] = true;
    }
    engine.onJointStates(partial);
    for (int i = 0; i < 50; ++i) {
        EXPECT_FALSE(engine.update(DT, (i + 1) * DT_NS).commanded);
    }
    EXPECT_FALSE(engine.initialized());
}

TEST(GaitEngine, StartsSeatedThenStandsWhenAsked) {
    Robot robot;
    robot.run(3.0);
    EXPECT_TRUE(robot.engine.initialized());
    EXPECT_EQ(robot.engine.sitState(), SitState::SEATED);

    const CommandResult refused = robot.engine.setMode(gait_core::MODE_TILT);
    EXPECT_FALSE(refused.success) << "tilt mode needs the robot standing";

    EXPECT_TRUE(robot.engine.sit(false).success);
    robot.run(6.0);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_NEAR(robot.engine.gaitConfig().currentHeight, robot.engine.gaitConfig().startPos.z(), tol(1e-9));
}

TEST(GaitEngine, WalkingForwardMovesTheOdometryForward) {
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);

    // 0.1 m/s for 4 s: about 0.4 m, less the time the gait takes to get going
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(4.0);
    const auto& odom = robot.engine.odometry();
    EXPECT_GT(odom.x(), 0.25);
    EXPECT_LT(odom.x(), 0.45);
    EXPECT_NEAR(odom.y(), 0.0, 0.02);
    EXPECT_NEAR(odom.yaw(), 0.0, 0.02);
}

TEST(GaitEngine, TurningLeftTurnsTheOdometryAnticlockwise) {
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);

    robot.engine.setVelocity({0, 0, static_cast<Scalar>(0.3)});
    robot.run(4.0);
    EXPECT_GT(robot.engine.odometry().yaw(), 0.5);
    EXPECT_NEAR(robot.engine.odometry().x(), 0.0, 0.03);
}

/**
 * @brief A robot that's been through startup and is standing
*/
Robot standingRobot(const GaitParams& params = GaitParams{}) {
    Robot robot(params);
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    return robot;
}

TEST(GaitEngine, ControllerTiltLatchesNeutralAndRecentresOnRelease) {
    Robot robot = standingRobot();
    ASSERT_TRUE(robot.engine.setMode(gait_core::MODE_TILT).success);

    ImuSample controller;
    controller.stamp = robot.now;
    controller.roll = static_cast<Scalar>(0.4);
    robot.engine.setControllerImu(controller, true);
    EXPECT_TRUE(robot.engine.controllerTiltActive());

    controller.stamp = robot.now + DT_NS;
    controller.roll = static_cast<Scalar>(0.5);
    robot.engine.setControllerImu(controller, true);
    robot.engine.setControllerImu(controller, false);
    EXPECT_FALSE(robot.engine.controllerTiltActive());
}

TEST(GaitEngine, ServosStayOffUntilTheFirstStand) {
    GaitParams params;
    params.power_on_startup = false;
    Robot robot(params);
    robot.run(3.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_EQ(robot.commandedTicks, 0) << "nothing is commanded while the servos are off";
    EXPECT_FALSE(robot.engine.initialized());

    const CommandResult stand = robot.engine.sit(false);
    EXPECT_TRUE(stand.success);
    EXPECT_NE(std::string(stand.message).find("powering"), std::string::npos) << stand.message;
    robot.run(9.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_TRUE(robot.engine.initialized());
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING) << "one stand command: power up, seated pose, stand";
}

TEST(GaitEngine, PowerDownSitsThenTurnsTheServosOff) {
    Robot robot = standingRobot();
    ASSERT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_TRUE(robot.engine.powerDown().success);
    robot.run(0.5);
    EXPECT_TRUE(robot.servoPower) << "still powered while sitting down";
    robot.run(8.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_TRUE(robot.engine.seated());
    EXPECT_NEAR(gait_core::YawPitchPitchLeg(explorerRobot().legs[0]).fk(robot.feedback.angles[0]).z(), -robot.engine.params().sit_height, 1e-3)
        << "the servos went off with the robot seated";

    const int commanded = robot.commandedTicks;
    robot.run(2.0);
    EXPECT_EQ(robot.commandedTicks, commanded);
    EXPECT_STREQ(robot.engine.powerDown().message, "servos already off");

    // Standing powers back up
    robot.engine.sit(false);
    robot.run(10.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, StandingUpBeforeTheServosGoOffCancelsThePowerDown) {
    Robot robot = standingRobot();
    robot.engine.powerDown();
    robot.run(1.0);
    robot.engine.sit(false);
    robot.run(10.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, LosingTheServosWaitsForAStandBeforeMovingAgain) {
    Robot robot = standingRobot();
    int seatedReports = 0;
    robot.engine.seatedChanged = [&seatedReports](bool seated) { seatedReports += seated ? 1 : 0; };

    // E-stop: the servos stop answering and the robot collapses
    robot.estop = true;
    robot.run(1.0);
    EXPECT_FALSE(robot.servoPower);
    EXPECT_TRUE(robot.engine.seated());
    EXPECT_EQ(seatedReports, 1);
    robot.collapse();

    // Releasing the e-stop moves nothing by itself
    robot.estop = false;
    const int commanded = robot.commandedTicks;
    robot.run(2.0);
    EXPECT_EQ(robot.commandedTicks, commanded);

    // A stand starts again from where the legs are now, through the seated pose
    robot.engine.sit(false);
    robot.run(0.3);
    EXPECT_TRUE(robot.engine.seated());
    robot.run(10.0);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, JointStatesThatKeepComingNeverCountAsAPowerLoss) {
    GaitParams params;
    params.servo_loss_timeout = static_cast<Scalar>(0.05);
    Robot robot = standingRobot(params);
    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(3.0);
    EXPECT_TRUE(robot.servoPower);
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
}

TEST(GaitEngine, EveryCommandIsReachable) {
    // Whatever the gait does, IK must never be asked for a pose outside the joint limits
    Robot robot;
    robot.run(3.0);
    robot.engine.sit(false);
    robot.run(6.0);
    robot.engine.setMode(gait_core::MODE_STRAFE);
    robot.engine.setVelocity({static_cast<Scalar>(0.08), static_cast<Scalar>(0.06), static_cast<Scalar>(0.3)});
    for (int i = 0; i < 400; ++i) {
        robot.run(0.01);
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const auto& q = robot.feedback.angles[leg];
            ASSERT_TRUE(q.allFinite()) << "leg " << leg;
            EXPECT_LE(std::abs(q(0)), 1.4 + 1e-6) << "leg " << leg;
            EXPECT_GE(q(1), -1.6 - 1e-6) << "leg " << leg;
            EXPECT_LE(q(1), 1.4 + 1e-6) << "leg " << leg;
        }
    }
}

// ---- Per-leg kinematics (JointVec) -----------------------------------------------------------

namespace {

/**
 * @brief The built in leg, counting how often the engine solves with it
*/
class CountingLeg : public gait_core::LegKinematics {
public:
    mutable int calls = 0;
    int numJoints() const override { return leg.numJoints(); }
    gait_core::Vec3 fk(const JointVec& q) const override { ++calls; return leg.fk(q); }
    bool ik(const gait_core::Vec3& target, const JointVec& seed, JointVec& q) const override {
        ++calls;
        return leg.ik(target, seed, q);
    }
    gait_core::LegJacobian jacobian(const JointVec& q) const override { return leg.jacobian(q); }
private:
    gait_core::YawPitchPitchLeg leg{explorerRobot().legs[0]};
};

}  // namespace

TEST(GaitEngine, JointVectorsStartSizedAndZeroNotEmpty) {
    // A default JointVec is empty, so a forgotten resize would hand callers zero-length vectors
    const JointFeedback feedback;
    const GaitOutput output;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(feedback.angles[leg], JointVec::Zero(3)) << "leg " << leg;
        EXPECT_EQ(feedback.effort[leg], JointVec::Zero(3)) << "leg " << leg;
        EXPECT_EQ(output.joints[leg], JointVec::Zero(3)) << "leg " << leg;
    }
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    const GaitOutput first = engine.update(DT, DT_NS);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(first.joints[leg], JointVec::Zero(3)) << "leg " << leg;
    }
}

TEST(GaitEngine, WrongSizedAnglesCountAsMissing) {
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    JointFeedback feedback;
    feedback.present.fill(true);
    feedback.angles[gait_core::explorer::LEG_REAR_LEFT] = JointVec::Zero(2);
    engine.onJointStates(feedback);
    for (int i = 0; i < 20; ++i) {
        EXPECT_FALSE(engine.update(DT, (i + 1) * DT_NS).commanded);
    }
}

TEST(GaitEngine, PowerLossRebuildKeepsLegKinematicsAndJointSizes) {
    const CountingLeg counting;   // outlives the robot, as setLegKinematics requires
    Robot robot = standingRobot();
    robot.engine.setLegKinematics(gait_core::explorer::LEG_FRONT_RIGHT, &counting);

    // Sit and cut the servos: cutServoPower rebuilds the engine and moves it into place
    ASSERT_TRUE(robot.engine.powerDown().success);
    robot.run(8.5);
    ASSERT_FALSE(robot.servoPower);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_EQ(robot.feedback.angles[leg].size(), 3) << "leg " << leg;
    }

    // Power up and stand: the override must still be in use, and every command finite and sized
    const int callsBefore = counting.calls;
    robot.engine.sit(false);
    for (int i = 0; i < 200 && robot.engine.sitState() != SitState::STANDING; ++i) {
        robot.run(0.1);   // up to 20 s to rise and gather the feet in
    }
    EXPECT_EQ(robot.engine.sitState(), SitState::STANDING);
    EXPECT_GT(counting.calls, callsBefore) << "the rebuilt engine dropped the leg's kinematics";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        ASSERT_EQ(robot.feedback.angles[leg].size(), 3) << "leg " << leg;
        EXPECT_TRUE(robot.feedback.angles[leg].allFinite()) << "leg " << leg;
    }

    // Null goes back to the built in leg
    robot.engine.setLegKinematics(gait_core::explorer::LEG_FRONT_RIGHT, nullptr);
    const int callsAfterReset = counting.calls;
    robot.run(0.5);
    EXPECT_EQ(counting.calls, callsAfterReset);
}

TEST(GaitEngine, CopiedEngineDoesNotUseTheOriginalsKinematics) {
    // The built in leg is reached through a null entry, not a pointer to the engine's own member,
    // so a copy keeps working after the original is gone
    std::unique_ptr<Robot> original = std::make_unique<Robot>(standingRobot());
    Robot copy = *original;
    original.reset();
    copy.engine.setVelocity({static_cast<Scalar>(0.05), 0, 0});
    copy.run(2.0);
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        EXPECT_TRUE(copy.feedback.angles[leg].allFinite()) << "leg " << leg;
    }
}
