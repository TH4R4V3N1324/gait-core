#ifndef GAIT_CORE__TEST__TEST_ROBOTS_HPP
#define GAIT_CORE__TEST__TEST_ROBOTS_HPP

// Robots that differ from explorer in one way each, to prove the engine isn't a hexapod engine:
// the leg count (a quadruped), and the joints per leg (four, and two). Only the tests use them.

#include <cmath>
#include <string>
#include <vector>

#include "test_common.hpp"
#include "gait_core/kinematics/chain_kinematics.hpp"
#include "gait_core/locomotion/gait_engine.hpp"

namespace test_robots {

using gait_core::GaitPattern;
using gait_core::JointDefinition;
using gait_core::LegDefinition;
using gait_core::LegSolver;
using gait_core::RobotDefinition;
using gait_core::Scalar;
using gait_core::Vec3;

inline Vec3 v(double x, double y, double z) {
    return Vec3(static_cast<Scalar>(x), static_cast<Scalar>(y), static_cast<Scalar>(z));
}

inline JointDefinition revolute(const Vec3& origin, const Vec3& axis, double lower, double upper) {
    return {origin, axis, static_cast<Scalar>(lower), static_cast<Scalar>(upper)};
}

/**
 * @brief Explorer's legs on the corners of a square body, crawling one leg at a time
 * @note Legs 0..3 front right, rear right, rear left, front left. A crawl is only stable with the
 *       body swayed over the three feet that stay down, so the engine sways it.
*/
inline RobotDefinition quadruped() {
    using gait_core::kPi;
    RobotDefinition robot = explorerRobot();
    const LegDefinition leg = robot.legs[0];
    const Scalar c = static_cast<Scalar>(0.12);
    const Scalar yaws[4] = {-kPi / 4, -3 * kPi / 4, 3 * kPi / 4, kPi / 4};
    const Scalar xs[4] = {c, -c, -c, c};
    const Scalar ys[4] = {-c, -c, c, c};
    robot.numLegs = 4;
    for (int i = 0; i < 4; ++i) {
        robot.legs[i] = leg;
        robot.legs[i].mount = {yaws[i], Vec3(xs[i], ys[i], 0)};
    }
    robot.numGaits = 1;
    // Rear right, front right, rear left, front left: each lift is next to the last one's side
    robot.gaits[0] = GaitPattern::fromGroups("crawl", {{1}, {0}, {2}, {3}});
    return robot;
}

/**
 * @brief Explorer with an ankle: the tibia split into a 0.100 m shank and a 0.086 m foot link by a
 *        pitch joint, so each leg has four joints for three dimensions
 * @note Same stance and reach as explorer. Not yaw-pitch-pitch, so it runs on ChainKinematics.
*/
inline RobotDefinition fourJointHexapod() {
    RobotDefinition robot = explorerRobot();
    for (int i = 0; i < robot.numLegs; ++i) {
        LegDefinition& leg = robot.legs[i];
        leg.numJoints = 4;
        leg.joints[3] = revolute(v(0.100, 0, 0), v(0, 1, 0), -1.2, 1.2);   // ankle, + folds the foot down
        leg.foot = v(0.086, 0, 0);
        leg.solver = LegSolver::Chain;
    }
    return robot;
}

/**
 * @brief Explorer with two joints per leg: a coxa yaw, then one pitch joint and a straight 0.176 m
 *        leg to the foot
 * @note Its foot moves on a surface: for each distance out from the hip there is one height. The
 *       stance (0.177 m out, 0.134 m down) is on it; most other heights aren't.
*/
inline RobotDefinition twoJointHexapod() {
    RobotDefinition robot = explorerRobot();
    for (int i = 0; i < robot.numLegs; ++i) {
        LegDefinition& leg = robot.legs[i];
        leg.numJoints = 2;
        leg.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), -1.4, 1.4);        // coxa
        leg.joints[1] = revolute(v(0.063, 0, 0), v(0, 1, 0), -1.6, 1.4);    // hip, + lowers the foot
        leg.foot = v(std::hypot(0.177 - 0.063, 0.134), 0, 0);
        leg.solver = LegSolver::Chain;
    }
    return robot;
}

/**
 * @brief Runs an engine on any robot with perfect servos: every command comes straight back as the
 *        joint state, while the engine wants the servos powered
 * @note Checks every command as it goes (sized to the leg, finite, inside the joint limits) and
 *       keeps the first problem, plus every warning and error the engine logs.
*/
class SimRobot {
public:
    explicit SimRobot(const RobotDefinition& robot, const gait_core::GaitParams& params = gait_core::GaitParams{})
        : definition(robot), engine(params, robot), servoPower(params.power_on_startup) {
        for (int leg = 0; leg < robot.numLegs; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = gait_core::JointVec::Zero(robot.legs[leg].numJoints);
        }
        engine.log = [this](gait_core::LogLevel level, const char* message) {
            if (level == gait_core::LogLevel::Warn) {
                warnings.emplace_back(stage + ": " + message);
            } else if (level == gait_core::LogLevel::Error) {
                errors.emplace_back(stage + ": " + message);
            }
        };
    }
    SimRobot(const SimRobot&) = delete;   // the log hook points at this one
    SimRobot& operator=(const SimRobot&) = delete;

    void run(double seconds) {
        const int ticks = static_cast<int>(std::lround(seconds / 0.01));
        for (int i = 0; i < ticks; ++i) {
            if (servoPower) {
                feedback.stamp = now;
                engine.onJointStates(feedback);
            }
            now += 10'000'000;
            const gait_core::GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            servoPower = out.servoPower;
            if (out.commanded) {
                for (int leg = 0; leg < definition.numLegs; ++leg) {
                    check(leg, out.joints[leg]);
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }

    /**
     * @brief Run until a condition holds, for at most some time
     * @return Whether it held
    */
    template <typename Condition>
    bool runUntil(Condition condition, double limit) {
        for (double t = 0; t < limit && !condition(); t += 0.1) {
            run(0.1);
        }
        return condition();
    }

    bool standUp() {
        run(3.0);
        engine.sit(false);
        return runUntil([this] { return engine.sitState() == gait_core::SitState::STANDING; }, 30.0);
    }

    // Where each foot is, from the last command (body frame)
    Vec3 foot(int leg) const {
        return engine.trajectory().legToBody(leg, gait_core::ChainKinematics(definition.legs[leg]).fk(feedback.angles[leg]));
    }

    RobotDefinition definition;
    gait_core::GaitEngine engine;
    gait_core::JointFeedback feedback;
    gait_core::Timestamp now = 1'000'000'000;
    bool servoPower;
    std::string problem;                 // the first bad command, empty if none
    std::string stage = "start";      // what the robot is doing, for messages
    std::vector<std::string> warnings;
    std::vector<std::string> errors;

private:
    void check(int leg, const gait_core::JointVec& q) {
        if (!problem.empty()) {
            return;
        }
        const LegDefinition& def = definition.legs[leg];
        const std::string where = "leg " + std::to_string(leg) + " at t=" + std::to_string(now / 1e9) + " s (" + stage + "): ";
        if (q.size() != def.numJoints) {
            problem = where + std::to_string(q.size()) + " joints commanded, not " + std::to_string(def.numJoints);
        } else if (!q.allFinite()) {
            problem = where + "not finite";
        } else {
            for (int j = 0; j < def.numJoints; ++j) {
                if (q(j) < def.joints[j].lower - 1e-6 || q(j) > def.joints[j].upper + 1e-6) {
                    problem = where + "joint " + std::to_string(j) + " at " + std::to_string(q(j)) + " rad, outside its limits";
                    return;
                }
            }
        }
    }
};

}  // namespace test_robots

#endif  // GAIT_CORE__TEST__TEST_ROBOTS_HPP
