#include <gtest/gtest.h>

#include <cmath>

#include "test_common.hpp"
#include "gait_core/gait_engine.hpp"

using gait_core::CommandResult;
using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::JointFeedback;
using gait_core::MAX_LEGS;
using gait_core::Scalar;
using gait_core::SitState;
using gait_core::Timestamp;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);  // s
constexpr Timestamp DT_NS = 10'000'000;

/**
 * @brief Runs an engine with perfect servos: every command comes straight back as the joint state
*/
class Robot {
public:
    GaitEngine engine;
    Timestamp now = 1'000'000'000;
    JointFeedback feedback;
    int commandedTicks = 0;

    Robot() {
        // Legs laid out flat at their zeros
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = {0, 0, 0};
        }
    }

    void run(double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        for (int i = 0; i < ticks; ++i) {
            feedback.stamp = now;
            engine.onJointStates(feedback);
            now += DT_NS;
            const GaitOutput out = engine.update(DT, now);
            if (out.commanded) {
                ++commandedTicks;
                for (int leg = 1; leg <= MAX_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }
};

}  // namespace

TEST(GaitEngine, WaitsForEveryJointBeforeMoving) {
    GaitEngine engine;
    JointFeedback partial;
    for (int leg = 1; leg < MAX_LEGS; ++leg) {  // leg 6 missing
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
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            const auto& q = robot.feedback.angles[leg];
            ASSERT_TRUE(std::isfinite(q.coxa) && std::isfinite(q.femur) && std::isfinite(q.tibia)) << "leg " << leg;
            EXPECT_LE(std::abs(q.coxa), 1.4 + 1e-6) << "leg " << leg;
            EXPECT_GE(q.femur, -1.6 - 1e-6) << "leg " << leg;
            EXPECT_LE(q.femur, 1.4 + 1e-6) << "leg " << leg;
        }
    }
}
