#include "gait_engine_internal.hpp"

// GaitEngine is one class split by area: this file builds it and holds the shared pieces
// (per-leg kinematics, logging, reach checks). The rest:
//   gait_engine_inputs.cpp    IMU, joint states, foot forces, leg odometry
//   gait_engine_commands.cpp  gait, mode, height, sit, power, body pose
//   gait_engine_tick.cpp      update(), startup, sending foot targets, sitting
//   gait_engine_walk.cpp      stepping, walking, returning to stance
//   gait_engine_terrain.cpp   terrain adaptation from foot forces
//   gait_engine_balance.cpp   tilt mode, load balancing, leveling while walking

// Moved from hexapod-ros2's gait_controller.cpp. The logic is unchanged, so the node's golden trace
// test still matches; only the ROS calls were swapped for the engine's inputs, outputs and hooks.

namespace gait_core {

using namespace engine_internal;

namespace {

/**
 * @brief The robot with numLegs kept within the per-leg arrays, since every leg loop trusts it
 * @note Only guards memory: a leg count the gaits can't schedule is a separate check.
*/
RobotDefinition withinCapacity(RobotDefinition robot) {
    robot.numLegs = std::clamp(robot.numLegs, 0, kMaxLegs);
    return robot;
}

/**
 * @brief Whether YawPitchPitchLeg (built from this leg) gives exactly this leg's kinematics
 * @param[in] leg The leg
 * @return true for a yaw joint then two pitch joints in a line, whatever the link lengths: a coxa about +z at the leg origin,
 *         then femur and tibia joints about +y spaced out along +x, and the foot out along +x
*/
bool fitsYawPitchPitch(const LegDefinition& leg) {
    if (leg.numJoints != 3) {
        return false;
    }
    auto along = [](const Vec3& v, const Vec3& direction) {
        return (v.normalized() - direction).norm() < S(1e-6);
    };
    auto outward = [](const Vec3& offset) {
        return offset.y() == 0 && offset.z() == 0 && offset.x() > 0;
    };
    return along(leg.joints[0].axis, Vec3::UnitZ()) && leg.joints[0].origin.isZero() &&
        along(leg.joints[1].axis, Vec3::UnitY()) && outward(leg.joints[1].origin) &&
        along(leg.joints[2].axis, Vec3::UnitY()) && outward(leg.joints[2].origin) &&
        outward(leg.foot);
}

}  // namespace

GaitEngine::GaitEngine(const GaitParams& params, const RobotDefinition& geometry)
    : p(params), robot(withinCapacity(geometry)), trajectoryGen(robot),
      config(robot), servo_power(params.power_on_startup) {
    if (p.initial_mode >= NUM_MODES) {
        p.initial_mode = MODE_NORMAL;
    }
    config.setMode(static_cast<Mode>(p.initial_mode));
    trajectoryGen.swingLift = p.step_height;
    trajectoryGen.swingEasing = p.swing_easing;
    target_height = config.currentHeight;
    stand_reach = config.startPos.x();
    stand_height = target_height;

    current_leg_positions.fill(Vec3::Zero());
    measured_leg_positions.fill(Vec3::Zero());
    startup_start_positions.fill(Vec3::Zero());
    swing_start.fill(Vec3::Zero());
    swing_target.fill(Vec3::Zero());
    tilt_ground_feet.fill(Vec3::Zero());
    tilt_saved_feet.fill(Vec3::Zero());
    previous_feet.fill(Vec2::Zero());
    foot_forces.fill(Vec3::Zero());
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        // Each leg's own solver: the analytic one only where it really is this leg's kinematics
        const LegDefinition& leg = robot.legs[legNum];
        chains[legNum] = ChainKinematics(leg);
        if (leg.solver == LegSolver::YawPitchPitch && fitsYawPitchPitch(leg)) {
            analytic[legNum].emplace(leg);
            leg_solver[legNum] = LegSolver::YawPitchPitch;
        } else {
            leg_solver[legNum] = LegSolver::Chain;
        }
        // A default JointVec is empty, not zero: size every leg's command to its joint count
        output.joints[legNum] = JointVec::Zero(kinematics(legNum).numJoints());
    }
}

/**
 * @brief Use other kinematics for one leg, e.g. an analytic solver for a different leg design
 * @param[in] legNum The leg, 0..numLegs() - 1; others are ignored
 * @param[in] kinematics Not owned, and must outlive the engine; null goes back to the built in
 *            YawPitchPitchLeg. Ignored, with a warning, if its numJoints() is not 1..kMaxJointsPerLeg.
 * @note Resets that leg's joint command to zeros of the new size, so set it before standing up.
 *       Kept when servo power is cut.
*/
void GaitEngine::setLegKinematics(int legNum, const LegKinematics* kinematics) {
    if (legNum < 0 || legNum >= numLegs()) {
        return;
    }
    if (kinematics != nullptr &&
        (kinematics->numJoints() < 1 || kinematics->numJoints() > kMaxJointsPerLeg)) {
        emit(LogLevel::Warn, "Leg %d kinematics ignored: %d joints, at most %d supported",
             legNum, kinematics->numJoints(), kMaxJointsPerLeg);
        return;
    }
    legKinematics[legNum] = kinematics;
    output.joints[legNum] = JointVec::Zero(this->kinematics(legNum).numJoints());
}

// ---------------------------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------------------------

void GaitEngine::emit(LogLevel level, const char* format, ...) {
    if (!log) {
        return;
    }
    char message[160];
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    log(level, message);
}

/**
 * @brief Whether a throttled message is due, marking it sent if so
*/
bool GaitEngine::throttleDue(int key, Scalar period) {
    if (throttled_log_sent[key] && toSeconds(now_ - last_throttled_log[key]) < static_cast<double>(period)) {
        return false;
    }
    throttled_log_sent[key] = true;
    last_throttled_log[key] = now_;
    return true;
}

void GaitEngine::emitThrottled(int key, Scalar period, LogLevel level, const char* format, ...) {
    if (!log || !throttleDue(key, period)) {
        return;
    }
    char message[160];
    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    log(level, message);
}

/**
 * @brief Whether a leg can put its foot at a target
 * @param[in] legNum The leg
 * @param[in] target The foot position (leg frame)
 * @return true if IK lands the foot within 1 mm of the target with every joint inside its limits
 * @note The limits are each joint's lower/upper in the leg's definition, so any number of joints
 *       works. A leg given kinematics (setLegKinematics) with a different joint count than its
 *       definition is never reachable, as there are no limits to check it against.
*/
bool GaitEngine::reachable(int legNum, const Vec3& target) const {
    constexpr Scalar REACH_TOLERANCE = S(1e-3);  // m
    const LegKinematics& leg = kinematics(legNum);              // the solver
    const LegDefinition& definition = robot.legs[legNum];  // its joints and their limits
    if (leg.numJoints() != definition.numJoints) {
        return false;  // an override with a different chain: no limits to judge it by
    }
    JointVec angles;
    leg.ik(target, output.joints[legNum], angles);  // the result is judged below, not by ik's flag
    for (int j = 0; j < leg.numJoints(); ++j) {
        if (angles(j) < definition.joints[j].lower || angles(j) > definition.joints[j].upper) {
            return false;
        }
    }
    return (leg.fk(angles) - target).norm() <= REACH_TOLERANCE;
}

/**
 * @brief Whether every leg can put its foot at a shared pose, e.g. seated or standing
 * @param[in] target The foot position (leg frame), before each leg's stanceOffset is added
*/
bool GaitEngine::everyLegReaches(const Vec3& target) const {
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!reachable(legNum, target + config.stanceOffset[legNum])) {
            return false;
        }
    }
    return true;
}

}  // namespace gait_core
