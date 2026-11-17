#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "test_common.hpp"
#include "gait_core/gait_config.hpp"
#include "gait_core/gait_engine.hpp"
#include "gait_core/robot_definition.hpp"
#include "gait_core/trajectory_generator.hpp"

using gait_core::GaitConfig;
using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::JointFeedback;
using gait_core::JointVec;
using gait_core::kMaxLegs;
using gait_core::kPi;
using gait_core::LegConfig;
using gait_core::LegJacobian;
using gait_core::LogLevel;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::Timestamp;
using gait_core::TrajectoryGenerator;
using Vector3d = gait_core::Vec3;

namespace {

constexpr Scalar DT = static_cast<Scalar>(0.01);  // s
constexpr Timestamp DT_NS = 10'000'000;

/**
 * @brief Explorer's links on a square body with four corner legs, clockwise from the front right
 * @note Legs 4 and 5 keep explorer's mounts, so anything that reads past numLegs would still
 *       find plausible numbers rather than crash; the tests check results, not just survival.
*/
RobotDefinition quadruped() {
    RobotDefinition robot = explorerRobot();
    robot.numLegs = 4;
    const Scalar c = static_cast<Scalar>(0.1);
    // Explorer's leg chains, on the corners of a square body
    robot.legs[0].mount = {-kPi / 4, Vector3d(c, -c, 0)};
    robot.legs[1].mount = {-3 * kPi / 4, Vector3d(-c, -c, 0)};
    robot.legs[2].mount = {3 * kPi / 4, Vector3d(-c, c, 0)};
    robot.legs[3].mount = {kPi / 4, Vector3d(c, c, 0)};
    return robot;
}

/**
 * @brief A leg with two joints, for checking which legs the engine sizes for which kinematics
*/
class TwoJointLeg : public gait_core::LegKinematics {
public:
    int numJoints() const override { return 2; }
    Vector3d fk(const JointVec&) const override { return Vector3d::Zero(); }
    bool ik(const Vector3d&, const JointVec&, JointVec& q) const override {
        q = JointVec::Zero(2);
        return true;
    }
    LegJacobian jacobian(const JointVec&) const override { return LegJacobian::Zero(3, 2); }
};

}  // namespace

TEST(RobotDefinition, ExplorerHasSixLegs) {
    EXPECT_EQ(explorerRobot().numLegs, 6);
    EXPECT_EQ(RobotDefinition{}.numLegs, 0) << "a robot must say how many legs it has";
    EXPECT_TRUE(GaitEngine(gait_core::GaitParams{}, explorerRobot()).robotSupported());
    EXPECT_EQ(GaitEngine(gait_core::GaitParams{}, explorerRobot()).numLegs(), 6);
}

TEST(RobotDefinition, QuadrupedLegsMapToTheirMounts) {
    const RobotDefinition robot = quadruped();
    const TrajectoryGenerator gen(robot);
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        const LegConfig& mount = robot.legs[leg].mount;
        // The coxa joint sits at the mount, and the leg's +x points along the mount's yaw
        EXPECT_NEAR((gen.legToBody(leg, Vector3d::Zero()) - mount.translation_offset).norm(), 0.0, tol(1e-9))
            << "leg " << leg;
        const Vector3d out = gen.legToBody(leg, Vector3d(1, 0, 0)) - mount.translation_offset;
        EXPECT_NEAR(out.x(), std::cos(mount.yaw), tol(1e-9)) << "leg " << leg;
        EXPECT_NEAR(out.y(), std::sin(mount.yaw), tol(1e-9)) << "leg " << leg;
    }
}

TEST(RobotDefinition, GaitConfigUsesTheRobotsLegCount) {
    const RobotDefinition robot = quadruped();
    GaitConfig config(robot);
    EXPECT_EQ(config.numLegs(), 4);
    const Scalar height = static_cast<Scalar>(-0.1);
    const Scalar reach = static_cast<Scalar>(0.15);
    config.setHeight(height);
    config.setReach(reach);
    for (int leg = 0; leg < 4; ++leg) {
        EXPECT_EQ(config.startPosition[leg], Vector3d(reach, 0, height)) << "leg " << leg;
    }
}

TEST(RobotDefinition, EngineRefusesALegCountTheGaitsCantSchedule) {
    GaitEngine engine(gait_core::GaitParams{}, quadruped());
    std::string logged;
    int errors = 0;
    engine.log = [&](LogLevel level, const char* message) {
        if (level == LogLevel::Error) {
            ++errors;
            logged = message;
        }
    };
    EXPECT_EQ(engine.numLegs(), 4);
    EXPECT_FALSE(engine.robotSupported());

    // Every joint reported, and powered: still never moves, and asks for the servos off
    JointFeedback feedback;
    feedback.present.fill(true);
    for (int i = 0; i < 300; ++i) {
        feedback.stamp = (i + 1) * DT_NS;
        engine.onJointStates(feedback);
        const GaitOutput out = engine.update(DT, (i + 1) * DT_NS);
        EXPECT_FALSE(out.commanded) << "tick " << i;
        EXPECT_FALSE(out.servoPower) << "tick " << i;
    }
    EXPECT_FALSE(engine.initialized());
    EXPECT_EQ(errors, 1) << "logged once per 5 s, not every tick";
    EXPECT_NE(logged.find("4 legs"), std::string::npos) << logged;
}

TEST(RobotDefinition, LegKinematicsOnlyForLegsTheRobotHas) {
    const TwoJointLeg twoJoints;
    GaitEngine engine(gait_core::GaitParams{}, quadruped());
    engine.setLegKinematics(3, &twoJoints);    // the last leg: taken
    engine.setLegKinematics(4, &twoJoints);    // past numLegs: ignored
    engine.setLegKinematics(-1, &twoJoints);   // ignored
    const GaitOutput out = engine.update(DT, DT_NS);
    EXPECT_EQ(out.joints[3].size(), 2);
    EXPECT_EQ(out.joints[4].size(), 3) << "a leg the robot doesn't have was given kinematics";
}

TEST(RobotDefinition, LegCountIsKeptWithinTheArrays) {
    RobotDefinition tooMany = explorerRobot();
    tooMany.numLegs = kMaxLegs + 3;
    EXPECT_EQ(GaitEngine(gait_core::GaitParams{}, tooMany).numLegs(), kMaxLegs);

    RobotDefinition negative = explorerRobot();
    negative.numLegs = -1;
    GaitEngine none(gait_core::GaitParams{}, negative);
    EXPECT_EQ(none.numLegs(), 0);
    EXPECT_FALSE(none.robotSupported());
    EXPECT_FALSE(none.update(DT, DT_NS).commanded);
}

// ---- Each leg's solver --------------------------------------------------------------------------

namespace {

RobotDefinition explorerOn(gait_core::LegSolver solver) {
    RobotDefinition robot = explorerRobot();
    for (int leg = 0; leg < robot.numLegs; ++leg) {
        robot.legs[leg].solver = solver;
    }
    return robot;
}

/**
 * @brief Runs an engine with perfect servos (each command comes back as the joint state)
 * @return The command of every tick
*/
std::vector<GaitOutput> run(GaitEngine& engine, int ticks) {
    std::vector<GaitOutput> outputs;
    JointFeedback feedback;
    feedback.present.fill(true);
    for (int i = 0; i < ticks; ++i) {
        feedback.stamp = (i + 1) * DT_NS;
        engine.onJointStates(feedback);
        const GaitOutput out = engine.update(DT, (i + 2) * DT_NS);
        if (out.commanded) {
            feedback.angles = out.joints;
        }
        outputs.push_back(out);
    }
    return outputs;
}

}  // namespace

TEST(LegSolver, ExplorerUsesTheAnalyticSolver) {
    const GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    for (int leg = 0; leg < engine.numLegs(); ++leg) {
        EXPECT_EQ(explorerRobot().legs[leg].solver, gait_core::LegSolver::YawPitchPitch) << "leg " << leg;
        EXPECT_EQ(engine.legSolver(leg), gait_core::LegSolver::YawPitchPitch) << "leg " << leg;
    }
}

TEST(LegSolver, ALegDefinedWithoutOneUsesItsChain) {
    gait_core::LegDefinition leg{};
    EXPECT_EQ(leg.solver, gait_core::LegSolver::Chain) << "the default must work for any leg";

    const GaitEngine engine(gait_core::GaitParams{}, explorerOn(gait_core::LegSolver::Chain));
    for (int i = 0; i < engine.numLegs(); ++i) {
        EXPECT_EQ(engine.legSolver(i), gait_core::LegSolver::Chain) << "leg " << i;
    }
}

TEST(LegSolver, TheAnalyticSolverOnlyWhereItFits) {
    // YawPitchPitchLeg is a coxa about z then femur and tibia about y, all along x: any lengths, but
    // anything else asking for it gets its chain instead of the wrong maths
    RobotDefinition robot = explorerRobot();
    robot.legs[0].joints[2].origin.x() += static_cast<Scalar>(0.01);   // a longer femur: still fits
    robot.legs[1].joints[1].axis = Vector3d::UnitX();                  // femur about the wrong axis
    robot.legs[2].numJoints = 4;                                        // an extra joint
    robot.legs[2].joints[3] = {robot.legs[2].foot, Vector3d::UnitY(), -1, 1};
    robot.legs[3].foot.z() = static_cast<Scalar>(-0.02);               // foot not straight out
    robot.legs[4].joints[1].origin.y() = static_cast<Scalar>(0.01);    // femur joint off to the side
    const GaitEngine engine(gait_core::GaitParams{}, robot);
    EXPECT_EQ(engine.legSolver(0), gait_core::LegSolver::YawPitchPitch);
    for (int leg = 1; leg <= 4; ++leg) {
        EXPECT_EQ(engine.legSolver(leg), gait_core::LegSolver::Chain) << "leg " << leg;
    }
    EXPECT_EQ(engine.legSolver(5), gait_core::LegSolver::YawPitchPitch);
}

TEST(LegSolver, ExplorerOnItsChainsStandsLikeTheAnalyticSolver) {
    // Startup and standing up, tick by tick: the numerical IK agrees with the analytic one to well
    // under a servo step (~1.5e-3 rad)
    GaitEngine analytic{gait_core::GaitParams{}, explorerRobot()};
    GaitEngine chain(gait_core::GaitParams{}, explorerOn(gait_core::LegSolver::Chain));
    const std::vector<GaitOutput> expected = run(analytic, 400);
    const std::vector<GaitOutput> actual = run(chain, 400);
    ASSERT_TRUE(expected.back().commanded);
    double worst = 0;
    size_t worstTick = 0;
    int worstLeg = 0;
    for (size_t i = 0; i < expected.size(); ++i) {
        ASSERT_EQ(actual[i].commanded, expected[i].commanded) << "tick " << i;
        for (int leg = 0; leg < analytic.numLegs(); ++leg) {
            ASSERT_EQ(actual[i].joints[leg].size(), 3);
            const double difference = static_cast<double>((actual[i].joints[leg] - expected[i].joints[leg]).cwiseAbs().maxCoeff());
            if (difference > worst) {
                worst = difference;
                worstTick = i;
                worstLeg = leg;
            }
        }
    }
    EXPECT_LT(worst, 1e-3) << "tick " << worstTick << " leg " << worstLeg << ": chain "
        << actual[worstTick].joints[worstLeg].transpose() << ", analytic " << expected[worstTick].joints[worstLeg].transpose();
    RecordProperty("worst_joint_difference_rad", std::to_string(worst));
}

// ---- Gaits from the robot -----------------------------------------------------------------------

namespace {

/**
 * @brief The square quadruped with gaits of its own: trot (diagonal pairs) and crawl
*/
RobotDefinition walkingQuadruped() {
    RobotDefinition robot = quadruped();
    robot.numGaits = 2;
    robot.gaits[0] = gait_core::GaitPattern::fromGroups("trot", {{0, 2}, {1, 3}});
    robot.gaits[1] = gait_core::GaitPattern::fromGroups("crawl", {{0}, {2}, {1}, {3}});
    return robot;
}

}  // namespace

TEST(RobotGaits, AnUnknownGaitIsRefused) {
    GaitEngine engine{gait_core::GaitParams{}, explorerRobot()};
    const gait_core::CommandResult result = engine.setGait(3);
    EXPECT_FALSE(result.success);
    EXPECT_NE(std::string(result.message).find("3 gaits"), std::string::npos) << result.message;
    EXPECT_FALSE(engine.gaitConfig().gaitChangeRequested);
    EXPECT_TRUE(engine.setGait(gait_core::explorer::GAIT_WAVE).success);
}

TEST(RobotGaits, AQuadrupedWithItsOwnGaitsIsSupported) {
    const GaitEngine engine(gait_core::GaitParams{}, walkingQuadruped());
    EXPECT_TRUE(engine.robotSupported());
    EXPECT_EQ(engine.gaitConfig().numGaits(), 2);
    EXPECT_STREQ(engine.gaitConfig().gaitName(1), "crawl");
}

TEST(RobotGaits, ARobotWithoutGaitsNeverMoves) {
    RobotDefinition robot = explorerRobot();
    robot.numGaits = 0;
    GaitEngine engine(gait_core::GaitParams{}, robot);
    EXPECT_FALSE(engine.robotSupported());
    EXPECT_FALSE(engine.update(DT, DT_NS).commanded);
}

TEST(RobotGaits, AQuadrupedStandsAndTrots) {
    // A smoke test: the engine runs a four-legged robot through startup and a few seconds of trot
    // with every command finite and sized. How well it walks is for later steps.
    GaitEngine engine(gait_core::GaitParams{}, walkingQuadruped());
    JointFeedback feedback;
    feedback.present.fill(true);
    int commanded = 0;
    for (int i = 0; i < 800; ++i) {
        if (i == 400) {
            engine.setVelocity({static_cast<Scalar>(0.03), 0, 0});
        }
        feedback.stamp = (i + 1) * DT_NS;
        engine.onJointStates(feedback);
        const GaitOutput out = engine.update(DT, (i + 2) * DT_NS);
        if (out.commanded) {
            ++commanded;
            for (int leg = 0; leg < 4; ++leg) {
                ASSERT_EQ(out.joints[leg].size(), 3) << "tick " << i << " leg " << leg;
                ASSERT_TRUE(out.joints[leg].allFinite()) << "tick " << i << " leg " << leg;
            }
            feedback.angles = out.joints;
        }
    }
    EXPECT_GT(commanded, 700);
    EXPECT_TRUE(engine.initialized());
}
