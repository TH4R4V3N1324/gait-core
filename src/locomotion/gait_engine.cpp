#include "gait_core/locomotion/gait_engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

// Moved from hexapod-ros2's gait_controller.cpp. The logic is unchanged, so the node's golden trace
// test still matches; only the ROS calls were swapped for the engine's inputs, outputs and hooks.

namespace gait_core {

namespace {

constexpr Scalar S(double value) {
    return static_cast<Scalar>(value);
}

/**
 * @brief printf into a command result's message
*/
void setMessage(CommandResult& result, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(result.message, sizeof(result.message), fmt, args);
    va_end(args);
}

// Keys for emitThrottled
constexpr int LOG_WAITING_FOR_JOINTS = 0;
constexpr int LOG_SWING_LOWERED = 1;
constexpr int LOG_TILT_LIMITED = 2;
constexpr int LOG_INVALID_MODE = 3;
constexpr int LOG_SERVOS_OFF = 4;
constexpr int LOG_UNSUPPORTED_ROBOT = 5;  // last_throttled_log and throttled_log_sent hold 6 keys

}  // namespace

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

// ---------------------------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------------------------

/**
 * @brief Store the body's orientation from the IMU
 * @param[in] sample Roll, pitch and yaw (rad), and when they were received
 * @note Yaw is only used by tilt mode to hold the heading; odometry provides it otherwise.
*/
void GaitEngine::setImu(const ImuSample& sample) {
    body_roll = sample.roll;
    body_pitch = sample.pitch;
    body_yaw = sample.yaw;
    last_imu_time = sample.stamp;
    has_imu = true;
}

void GaitEngine::setControllerImu(const ImuSample& sample, bool active) {
    if (!active) {
        if (controller_tilt_active) {
            setBodyPose(0, 0, 0, Vec3::Zero());
        }
        controller_tilt_active = false;
        return;
    }

    if (!controller_tilt_active) {
        controller_reference_roll = sample.roll;
        controller_reference_pitch = sample.pitch;
        controller_reference_yaw = sample.yaw;
        controller_tilt_active = true;
    }

    setBodyPose(
        std::remainder(sample.roll - controller_reference_roll, 2 * kPi),
        std::remainder(sample.pitch - controller_reference_pitch, 2 * kPi),
        std::remainder(sample.yaw - controller_reference_yaw, 2 * kPi),
        Vec3::Zero());
    last_controller_imu_time = sample.stamp;
}

bool GaitEngine::imuFresh() const {
    return has_imu && toSeconds(now_ - last_imu_time) < 0.2;
}

bool GaitEngine::controllerImuFresh() const {
    return controller_tilt_active && toSeconds(now_ - last_controller_imu_time) < 0.2;
}

/**
 * @brief Take a joint state: each foot's measured position, the foot forces and the leg odometry
 * @param[in] feedback The joint state
 * @return true if the leg odometry was updated
 * @note has_joint_states is only set once every joint has been received. A leg whose angles
 *       aren't sized to its kinematics' numJoints() counts as not received.
*/
bool GaitEngine::onJointStates(const JointFeedback& feedback) {
    bool all_found = true;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const LegKinematics& leg = kinematics(legNum);
        if (!feedback.present[legNum] || feedback.angles[legNum].size() != leg.numJoints()) {
            all_found = false;
            continue;
        }
        measured_leg_positions[legNum] = leg.fk(feedback.angles[legNum]);
    }
    has_joint_states = all_found;
    full_state_since_update = full_state_since_update || all_found;
    if (all_found && feedback.hasEffort) {
        updateFootForces(feedback);
    }

    if (has_joint_states && positions_initialized) {
        return updateLegOdometry(feedback.stamp);
    }
    return false;
}

/**
 * @brief Estimate the force on each foot from its joint efforts and decide which feet are in contact
 * @param[in] feedback The joint state, with efforts
 * @note Forces are in the body frame and low-pass filtered (foot_force_filter). Contact switches
 *       on above contact_force_on of upward force and off below contact_force_off.
*/
void GaitEngine::updateFootForces(const JointFeedback& feedback) {
    Scalar alpha = 1;
    if (has_foot_forces) {
        const Scalar dt = static_cast<Scalar>(std::max(0.0, toSeconds(feedback.stamp - last_foot_force_stamp)));
        alpha = p.foot_force_filter > 0 ? std::min(S(1.0), dt / p.foot_force_filter) : S(1.0);
    }
    last_foot_force_stamp = feedback.stamp;

    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        foot_force_updated[legNum] = false;
        Vec3 legForce;
        if (!kinematics(legNum).groundForce(feedback.angles[legNum], feedback.effort[legNum], legForce)) {
            continue;  // leg nearly straight, or efforts the wrong size: keep the last estimate
        }
        // Rotate into the body frame (a direction, so without the coxa offset)
        Vec3 bodyForce = trajectoryGen.legToBody(legNum, legForce) - trajectoryGen.legToBody(legNum, Vec3::Zero());
        // Mid-swing the foot is certainly in the air, so whatever it reads is the leg's own weight
        if (leg_swinging[legNum] && phase_progress > S(0.3) && phase_progress < S(0.7)) {
            foot_force_offset[legNum] += S(0.02) * (-bodyForce.z() - foot_force_offset[legNum]);
        }
        bodyForce.z() += foot_force_offset[legNum];
        foot_forces[legNum] = has_foot_forces ? Vec3(foot_forces[legNum] + alpha * (bodyForce - foot_forces[legNum])) : bodyForce;

        if (foot_forces[legNum].z() > p.contact_force_on) {
            foot_contact[legNum] = true;
        } else if (foot_forces[legNum].z() < p.contact_force_off) {
            foot_contact[legNum] = false;
        }
        foot_force_updated[legNum] = true;
    }
    has_foot_forces = true;

    // Blocked: the ground's net horizontal push on the feet in contact is a large share of the
    // weight, so something else is pushing back (walking freely it's ~0.07, against a wall ~0.5).
    // Filtered over ~0.3 s, with hysteresis.
    Vec2 horizontal = Vec2::Zero();
    Scalar vertical = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (foot_contact[legNum]) {
            horizontal += foot_forces[legNum].head<2>();
            vertical += foot_forces[legNum].z();
        }
    }
    const Scalar blockedAlpha = std::min(S(1.0), alpha * p.foot_force_filter / S(0.3));
    push_ratio += blockedAlpha * ((vertical > 3 * p.contact_force_on ? horizontal.norm() / vertical : S(0.0)) - push_ratio);
    const bool wasBlocked = is_blocked;
    if (push_ratio > p.blocked_force_ratio) {
        is_blocked = true;
    } else if (push_ratio < S(0.67) * p.blocked_force_ratio) {
        is_blocked = false;
    }
    if (is_blocked != wasBlocked) {
        emit(LogLevel::Info, is_blocked ? "Pushing against something: odometry ignores the legs' movement"
                                        : "No longer pushing against anything");
    }
}

/**
 * @brief Estimate the body's motion from the planted feet and add it to the leg odometry
 * @param[in] stamp The joint state timestamp
 * @return false if the timestamp was no newer than the last one, so nothing changed
 * @note Feet that are not swinging and are bearing weight are treated as planted. Also tracks the
 *       body's height from how far the planted feet move up or down, for floorRise.
*/
bool GaitEngine::updateLegOdometry(Timestamp stamp) {
    // In simulation the joint states come faster than the clock ticks, so ~30% share a timestamp
    // with the previous one. Skip those entirely, keeping the last feet, so their motion is
    // counted with the next new timestamp; replacing the feet threw it away (odometry read 20% short).
    if (has_previous_feet && stamp <= previous_feet_stamp) {
        return false;
    }
    std::array<Vec2, kMaxLegs> feet;
    feet.fill(Vec2::Zero());
    std::array<Scalar, kMaxLegs> feetZ{};
    const Eigen::Quaternion<Scalar> tiltNow = imuTilt();
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const Vec3 bodyFoot = trajectoryGen.legToBody(legNum, measured_leg_positions[legNum]);
        // In the level frame, so tilting the body to level it doesn't count as movement
        feet[legNum] = trajectoryGen.unposedBodyPoint(bodyFoot, walk_level).head<2>();
        // Heights levelled by the body's real tilt rather than the commanded leveling: rocking
        // while walking moves the feet up and down relative to the body without it climbing
        feetZ[legNum] = (tiltNow * bodyFoot).z();
    }

    if (has_previous_feet) {
        // Only feet bearing weight, when the joint efforts say so, since a foot only stays put once
        // it's loaded. Without effort data (mock hardware) every non-swinging foot counts.
        Scalar totalLoad = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            totalLoad += std::max(S(0.0), foot_forces[legNum].z());
        }
        const bool useContact = has_foot_forces && totalLoad > 3 * p.contact_force_on;
        const Scalar dt = static_cast<Scalar>(toSeconds(stamp - previous_feet_stamp));
        std::array<Vec2, kMaxLegs> before, after;
        std::array<Scalar, kMaxLegs> beforeZ, afterZ;
        size_t plantedCount = 0, settledCount = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            const bool planted = !isSwinging(legNum) && (!useContact || (foot_contact[legNum] && previous_contact[legNum]));
            if (planted) {
                before[plantedCount] = previous_feet[legNum];
                after[plantedCount] = feet[legNum];
                ++plantedCount;
            }
            // The servos lag at touchdown, so a foot the gait calls planted is still settling onto
            // the ground for a moment, and counting that drop as the body rising made the height
            // creep up a few mm a step. Only feet the forces say are loaded: counting every foot
            // without them took the spawn drop's bounce in simulation as the body sinking 4-5 cm.
            planted_time[legNum] = planted && useContact ? planted_time[legNum] + dt : S(0.0);
            if (planted_time[legNum] > CLIMB_SETTLE_TIME) {
                beforeZ[settledCount] = previous_feet_z[legNum];
                afterZ[settledCount] = feetZ[legNum];
                ++settledCount;
            }
        }
        BodyMotion motion;
        if (LegOdometry::estimateMotion(before.data(), after.data(), plantedCount, motion)) {
            // Pushing against something, the feet scrub along the ground, so they say the body is
            // moving when it isn't: keep only the turn
            legOdometry.integrate(is_blocked ? BodyMotion{0, 0, motion.dyaw} : motion);
        }
        if (climb_anchored) {
            Scalar climb;
            if (LegOdometry::estimateClimb(beforeZ.data(), afterZ.data(), settledCount, climb)) {
                legOdometry.integrateClimb(climb);
            }
        } else {
            // The floor the robot settles on is z = 0, with the body's height above it measured from
            // the feet that have been carrying weight for a while (at least a tripod)
            Scalar sum = 0;
            int settled = 0;
            for (int legNum = 0; legNum < numLegs(); ++legNum) {
                if (planted_time[legNum] > CLIMB_ANCHOR_TIME) {
                    sum += feetZ[legNum];
                    ++settled;
                }
            }
            if (settled >= 3) {
                odom_start_height = -sum / settled - legOdometry.z();
                climb_anchored = true;
                emit(LogLevel::Info, "Leg odometry height started: body %.1f mm above the floor, from %d feet",
                    static_cast<double>(-sum / settled * 1000), settled);
            }
        }
    }
    previous_feet = feet;
    previous_feet_z = feetZ;
    previous_contact = foot_contact;
    previous_feet_stamp = stamp;
    has_previous_feet = true;
    return true;
}

/**
 * @brief How far the body is above the floor, as the footprint transform has it
 * @return The commanded body height (m) plus any lift from tilt mode; it doesn't know about
 *         climbing (floorRise does)
*/
Scalar GaitEngine::footprintHeight() const {
    return -config.currentHeight + tilt_applied.translation.z();
}

/**
 * @brief The body's roll and pitch from the IMU, as a rotation from the body to the level frame
*/
Eigen::Quaternion<Scalar> GaitEngine::imuTilt() const {
    return Eigen::Quaternion<Scalar>(AngleAxis(body_pitch, Vec3::UnitY()) * AngleAxis(body_roll, Vec3::UnitX()));
}

/**
 * @brief How far the floor under the body has risen since the leg odometry started
 * @return The leg odometry's body height less the commanded body height (m); 0 until at least
 *         three feet have carried weight long enough to measure the body's height
*/
Scalar GaitEngine::floorRise() const {
    if (!climb_anchored) {
        return 0;
    }
    return odom_start_height + legOdometry.z() - footprintHeight();
}

// ---------------------------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------------------------

CommandResult GaitEngine::setGait(uint8_t gait) {
    CommandResult result;
    // An index past the robot's list would give an empty schedule, and phase % 0
    if (gait >= config.numGaits()) {
        setMessage(result, "Gait %d is not one of this robot's %d gaits", gait, config.numGaits());
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    config.setGait(gait);
    emit(LogLevel::Info, "Gait %d (%s) requested", gait, config.gaitName(gait));
    result.success = true;
    setMessage(result, "Gait %d (%s) requested, applies at the next step", gait, config.gaitName(gait));
    return result;
}

CommandResult GaitEngine::setMode(uint8_t mode) {
    CommandResult result;
    if (mode >= NUM_MODES) {
        setMessage(result, "Mode %d is not a valid mode", mode);
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    if (mode == MODE_TILT && sit_state != SitState::STANDING) {
        setMessage(result, "Can't enter tilt mode while sitting");
        emit(LogLevel::Warn, "%s", result.message);
        return result;
    }
    config.setMode(static_cast<Mode>(mode));
    result.success = true;
    setMessage(result, "Mode changed to %d", mode);
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief Glide the body to a new height
 * @param[in] height The feet's z below the hips, so negative: -0.12 for a 12 cm body height
*/
CommandResult GaitEngine::setHeight(Scalar height) {
    CommandResult result;
    const double h = static_cast<double>(height);
    if (sit_state != SitState::STANDING) {
        setMessage(result, "can't change height while sitting");
        emit(LogLevel::Warn, "Refused height %.3f: %s", h, result.message);
        return result;
    }
    if (-height < p.min_height || -height > p.max_height) {
        setMessage(result, "height must be between %f and %f m (sent as negative z)",
            static_cast<double>(p.min_height), static_cast<double>(p.max_height));
        emit(LogLevel::Warn, "Refused height %.3f: %s", h, result.message);
        return result;
    }
    target_height = height;
    result.success = true;
    setMessage(result, "moving to %f m", -h);
    emit(LogLevel::Info, "Height target %.3f m", -h);
    return result;
}

/**
 * @brief Sit down (true) or stand up (false)
 * @note Returns straight away; the move carries on in update(). A request part way through
 *       reverses the move from wherever it has got to.
*/
CommandResult GaitEngine::sit(bool down) {
    CommandResult result;
    result.success = true;
    if (down) {
        switch (sit_state) {
            case SitState::STANDING:
                if (!everyLegReaches(Vec3(p.sit_reach, 0, -p.sit_height)) ||
                    !everyLegReaches(Vec3(p.sit_reach, 0, stand_height))) {
                    result.success = false;
                    setMessage(result, "sit_reach and sit_height put a foot out of reach");
                    emit(LogLevel::Warn, "Refused to sit: %s", result.message);
                    return result;
                }
                stand_height = target_height;
                // Tilt mode eases the body back to neutral before the sit starts
                config.setMode(MODE_NORMAL);
                config.setReach(p.sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::GATHERING:
                config.setReach(p.sit_reach);
                sit_state = SitState::SPREADING;
                break;
            case SitState::RAISING:
                target_height = -p.sit_height;
                sit_state = SitState::LOWERING;
                break;
            case SitState::SPREADING:
            case SitState::LOWERING:
            case SitState::SEATED:
                setMessage(result, "already sitting");
                return result;
        }
        setMessage(result, "sitting down");
    } else {
        // Standing up cancels a power down; unpowered, it powers the servos and startup takes it from there
        power_down_pending = false;
        const bool poweringUp = !servo_power;
        if (poweringUp) {
            servo_power = true;
            has_joint_states = false;   // wait for joint states from the powered servos
        }
        switch (sit_state) {
            case SitState::SEATED:
            case SitState::LOWERING:
                target_height = stand_height;
                sit_state = SitState::RAISING;
                if (poweringUp) {
                    setMessage(result, "powering the servos, then standing up");
                    emit(LogLevel::Info, "%s", result.message);
                    return result;   // seated() hasn't changed: startup reports it once it stands
                }
                break;
            case SitState::SPREADING:
                config.setReach(stand_reach);
                sit_state = SitState::GATHERING;
                break;
            case SitState::STANDING:
            case SitState::RAISING:
            case SitState::GATHERING:
                setMessage(result, "already standing");
                return result;
        }
        setMessage(result, "standing up");
    }
    if (seatedChanged) {
        seatedChanged(seated());
    }
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief Sit down, then turn the servo power off once seated
 * @note A stand command before it's off cancels it; one after powers the servos back up.
*/
CommandResult GaitEngine::powerDown() {
    CommandResult result;
    if (!servo_power) {
        result.success = true;
        setMessage(result, "servos already off");
        return result;
    }
    const CommandResult sat = sit(true);
    if (!sat.success) {
        return sat;
    }
    power_down_pending = true;
    result.success = true;
    setMessage(result, sit_state == SitState::SEATED && positions_initialized
        ? "turning the servos off" : "sitting down, then turning the servos off");
    emit(LogLevel::Info, "%s", result.message);
    return result;
}

/**
 * @brief The servos lost power without being asked (e-stop, kill loop, brownout)
 * @note Turns servo power off, so nothing moves when the power comes back until a stand command.
 *       The engine notices by itself after servo_loss_timeout without joint states; call this when
 *       the hardware knows sooner.
*/
void GaitEngine::servoPowerLost() {
    if (!servo_power) {
        return;
    }
    emit(LogLevel::Warn, "Servo power lost: send a stand command to power up and stand again");
    cutServoPower();
}

/**
 * @brief Turn servo power off and go back to how the engine starts, so the next power up runs
 *        startup again from wherever the legs are
 * @note Keeps the settings, hooks, odometry, IMU, gait choice and standing height.
*/
void GaitEngine::cutServoPower() {
    const bool wasSeated = seated();
    GaitEngine fresh(p, robot);
    fresh.log = std::move(log);
    fresh.seatedChanged = std::move(seatedChanged);
    fresh.footholdPlanner = footholdPlanner;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        // Also sizes fresh.output.joints[legNum] for an override with a different joint count
        fresh.setLegKinematics(legNum, legKinematics[legNum]);
    }
    fresh.legOdometry = legOdometry;
    fresh.body_roll = body_roll;
    fresh.body_pitch = body_pitch;
    fresh.body_yaw = body_yaw;
    fresh.last_imu_time = last_imu_time;
    fresh.has_imu = has_imu;
    fresh.stand_height = stand_height;
    fresh.config.setGait(config.currentGait);
    fresh.last_throttled_log = last_throttled_log;
    fresh.throttled_log_sent = throttled_log_sent;
    fresh.now_ = now_;
    fresh.servo_power = false;
    *this = std::move(fresh);
    if (!wasSeated && seatedChanged) {
        seatedChanged(true);
    }
}

/**
 * @brief The body pose wanted in tilt mode, relative to level and the heading when tilt mode started
 * @note Each axis is clamped to tilt_max_translation or tilt_max_angle. Ignored outside tilt mode.
*/
void GaitEngine::setBodyPose(Scalar roll, Scalar pitch, Scalar yaw, const Vec3& translation) {
    body_pose_request.roll = std::clamp(roll, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.pitch = std::clamp(pitch, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.yaw = std::clamp(yaw, -p.tilt_max_angle, p.tilt_max_angle);
    body_pose_request.translation = translation.cwiseMax(-p.tilt_max_translation).cwiseMin(p.tilt_max_translation);
}

/**
 * @brief Fastest a foot can travel relative to the body in the current gait
 * @return The speed (m/s), which is also the gait's top walking speed
 * @note A stance foot sweeps up to 2 x maxStride over (groups - 1) phases, and phases can be no
 *       shorter than min_phase_duration
*/
Scalar GaitEngine::maxFootSpeed() const {
    size_t groups = std::max<size_t>(2, config.getGaitConfig(config.currentGait).size());
    return 2 * config.maxStride / ((groups - 1) * p.min_phase_duration);
}

// ---------------------------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------------------------

GaitOutput GaitEngine::update(Scalar dt, Timestamp now) {
    now_ = now;
    // A leg count the gait schedules can't handle: never command, and keep the servos off
    if (!robotSupported()) {
        const int gait = config.unrunnableGait();
        if (config.numGaits() == 0) {
            emitThrottled(LOG_UNSUPPORTED_ROBOT, 5, LogLevel::Error, "Robot has no gaits: it won't move");
        } else {
            emitThrottled(LOG_UNSUPPORTED_ROBOT, 5, LogLevel::Error,
                "Gait %d (%s) can't run on this robot's %d legs: it won't move", gait, config.gaitName(static_cast<uint8_t>(gait)), numLegs());
        }
        output.commanded = false;
        output.servoPower = false;
        return output;
    }
    if (controller_tilt_active && !controllerImuFresh()) {
        setControllerImu(ImuSample{}, false);
        emit(LogLevel::Warn, "Controller orientation timed out: tilt pose recentred");
    }
    if (full_state_since_update) {
        last_full_state = now;
        full_state_since_update = false;
    }

    // Servos that stopped answering lost their power (e-stop): don't move when it comes back
    if (servo_power && positions_initialized && p.servo_loss_timeout > 0 &&
        toSeconds(now - last_full_state) > static_cast<double>(p.servo_loss_timeout)) {
        emit(LogLevel::Warn, "No joint states for %.1f s: servo power lost (e-stop?). "
            "Send a stand command to power up and stand again", toSeconds(now - last_full_state));
        cutServoPower();
    }
    // Powering down: off once seated
    if (power_down_pending && positions_initialized && sit_state == SitState::SEATED) {
        emit(LogLevel::Info, "Seated: servos off");
        cutServoPower();
    }

    output.commanded = false;
    output.servoPower = servo_power;
    trajectoryGen.swingLift = p.step_height;
    trajectoryGen.swingEasing = p.swing_easing;

    // Run the startup sequence until the legs reach their start positions
    if (!positions_initialized) {
        positions_initialized = startup(dt);
        return output;
    }

    updateLoadBalance(dt);

    // Glide towards the height setHeight asked for, moving every foot by the same amount. Planted
    // feet follow the start positions; swings, the last commanded positions (which a swing restarts
    // from) and tilt mode's ground points carry their own heights, so they're shifted too.
    if (config.currentHeight != target_height) {
        const Scalar step = p.height_rate * dt;
        const Scalar delta = std::clamp(target_height - config.currentHeight, -step, step);
        config.setHeight(config.currentHeight + delta);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            current_leg_positions[legNum].z() += delta;
            swing_start[legNum].z() += delta;
            swing_target[legNum].z() += delta;
            tilt_ground_feet[legNum].z() += delta;
        }
    }

    // Tilt mode, or easing the body back to neutral after leaving it
    if (config.currentMode == MODE_TILT || tilt_active) {
        tilt(dt, config.currentMode != MODE_TILT);
        return output;
    }

    if (sit_state != SitState::STANDING) {
        sitTick(dt);
        return output;
    }

    switch (config.currentMode) {
        case MODE_STRAFE:
            current_motion_intent.forward = velocity.vx;
            current_motion_intent.left = velocity.vy;
            current_motion_intent.yaw = velocity.wz;
            break;
        case MODE_NORMAL:
            // No strafing in this mode. Turning comes from wz (as Nav2 sends it) plus vy, scaled so
            // the gamepad's full linear range maps to the full yaw range
            current_motion_intent.forward = velocity.vx;
            current_motion_intent.left = 0;
            current_motion_intent.yaw = velocity.wz + velocity.vy / trajectoryGen.maxBodyRadius();
            break;
        case MODE_TILT:  // handled by tilt() above
        case NUM_MODES:
        default:
            emitThrottled(LOG_INVALID_MODE, 1, LogLevel::Warn, "Invalid or unimplemented mode");
            break;
    }
    updateWalkLeveling(dt);
    walk(dt);
    return output;
}

/**
 * @brief Solve IK for a leg target and record it as that leg's joint command
 * @param[in] target The foot target position (leg frame)
 * @param[in] legNum The leg
 * @note The body is leveled while walking and each foot raised or lowered by load balancing.
 *       Also records target as the leg's current position.
*/
void GaitEngine::commandFoot(const Vec3& target, int legNum) {
    // ik doesn't promise to read the seed before writing the result, so they can't be one vector.
    // Out of reach gives the closest pose, as before; ik's flag is unused for now.
    const JointVec seed = output.joints[legNum];
    kinematics(legNum).ik(trajectoryGen.posedFoot(legNum, target, walk_level)
        + Vec3(0, 0, foot_height_offset[legNum]), seed, output.joints[legNum]);
    current_leg_positions[legNum] = target;
}

/**
 * @brief Command every foot towards its target, limited to max_foot_speed
 * @param[in] targets The foot target of each leg (leg frame)
 * @param[in] dt The time since the last tick (s)
*/
void GaitEngine::sendFootTargets(const std::array<Vec3, kMaxLegs>& targets, Scalar dt) {
    const Scalar maxStep = p.max_foot_speed * dt;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        Vec3 targetPos = targets[legNum];
        const Vec3 delta = targetPos - current_leg_positions[legNum];
        if (delta.norm() > maxStep) {
            targetPos = current_leg_positions[legNum] + delta.normalized() * maxStep;
        }
        commandFoot(targetPos, legNum);
    }
    output.commanded = true;
}

/**
 * @brief Startup sequence to move legs from where they are to the seated pose
 * @param[in] dt The time since the last tick (s)
 * @return true once the sequence is complete, false while it is still running
 * @note Waits for joint states first, so the move starts from the measured leg positions instead
 *       of assuming a pose. The move takes startup_duration, or longer if a joint would otherwise
 *       exceed max_joint_speed. The robot is then seated, and stands up if stand_on_startup is set
 *       (or a stand was asked for meanwhile); otherwise it waits for one.
*/
bool GaitEngine::startup(Scalar dt) {
    if (!servo_power) {
        emitThrottled(LOG_SERVOS_OFF, 30, LogLevel::Info, "Servos off: send a stand command to power them up");
        return false;
    }
    if (!startup_planned) {
        if (!has_joint_states) {
            emitThrottled(LOG_WAITING_FOR_JOINTS, 5, LogLevel::Info, "Waiting for joint states from every joint");
            return false;
        }
        // Into the seated pose, whatever pose the legs start in: the stand up from there is the same
        // as after a sit request
        if (everyLegReaches(Vec3(p.sit_reach, 0, -p.sit_height)) && everyLegReaches(Vec3(p.sit_reach, 0, stand_height))) {
            config.setReach(p.sit_reach);
            config.setHeight(-p.sit_height);
            target_height = -p.sit_height;
            emit(LogLevel::Info, "Moving legs from their measured positions to the seated pose");
        } else {
            sit_state = SitState::STANDING;
            emit(LogLevel::Warn, "sit_reach and sit_height put a foot out of reach: "
                "moving legs from their measured positions straight to the standing pose");
        }
        current_leg_positions = measured_leg_positions;
        startup_start_positions = measured_leg_positions;
        startup_progress = 0;
        startup_planned = true;
    }

    phase = 0;
    config.setHeight(config.currentHeight);

    // Straight line from the measured position to the start position
    auto targetsAt = [this](Scalar progress) {
        std::array<Vec3, kMaxLegs> targets;
        for (int leg = 0; leg < numLegs(); ++leg) {
            const Vec3& start = startup_start_positions[leg];
            targets[leg] = start + (config.startPosition.at(leg) - start) * progress;
        }
        return targets;
    };

    const Scalar ds = dt / p.startup_duration;
    const Scalar scale = jointSpeedScale(targetsAt(std::min(S(1.0), startup_progress + ds)), dt);
    startup_progress = std::min(S(1.0), startup_progress + ds * scale);

    const auto targets = targetsAt(startup_progress);
    for (int leg = 0; leg < numLegs(); ++leg) {
        commandFoot(targets[leg], leg);
    }
    output.commanded = true;
    // Where the feet were sent: standing up from the seated pose holds them here
    current_leg_positions = targets;

    if (startup_progress >= 1) {
        emit(LogLevel::Info, "Startup sequence complete");
        // Stand straight away if asked to, or if a stand request came in during the move
        if (sit_state != SitState::STANDING && (p.stand_on_startup || sit_state == SitState::RAISING)) {
            target_height = stand_height;
            sit_state = SitState::RAISING;
            emit(LogLevel::Info, "Standing up");
        } else if (sit_state == SitState::SEATED) {
            emit(LogLevel::Info, "Seated: call sit with false to stand up");
        }
        if (seatedChanged) {
            seatedChanged(seated());
        }
        return true;
    }
    return false;
}

/**
 * @brief Evaluate every leg's trajectory target after advancing it by ds, without changing the splines
 * @param[in] ds How far to advance each spline (fraction of a phase)
 * @return The target foot position of each leg (leg frame)
 * @note Legs with no active swing or stance spline hold their commanded position
*/
std::array<Vec3, kMaxLegs> GaitEngine::legTargets(Scalar ds) {
    std::array<Vec3, kMaxLegs> targets;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        const LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];

        if (spline.active) {
            const Scalar s = std::min(S(1.0), spline.s + ds);
            targets[legNum] = (s >= 1) ? spline.P4
                : limitSwingLift(legNum, trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s),
                                 spline.P0.z() + (spline.P4.z() - spline.P0.z()) * s);
        } else if (stanceSpline.active) {
            const Scalar s = std::min(S(1.0), stanceSpline.s + ds);
            targets[legNum] = stanceSpline.P0 + (stanceSpline.P1 - stanceSpline.P0) * s;
        } else {
            targets[legNum] = current_leg_positions[legNum];
        }
    }
    return targets;
}

/**
 * @brief Work out how much to slow this tick so no joint moves faster than max_joint_speed
 * @param[in] targets The foot target of each leg for this tick (leg frame)
 * @param[in] dt The time since the last tick (s)
 * @return A factor from 0 to 1 to scale this tick's progress by (1 = full speed)
*/
Scalar GaitEngine::jointSpeedScale(const std::array<Vec3, kMaxLegs>& targets, Scalar dt) {
    Scalar max_delta = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        const LegKinematics& leg = kinematics(legNum);
        JointVec from;
        JointVec to;
        leg.ik(current_leg_positions[legNum], output.joints[legNum], from);
        leg.ik(targets[legNum], from, to);
        max_delta = std::max(max_delta, (to - from).cwiseAbs().maxCoeff());
    }
    const Scalar allowed = p.max_joint_speed * dt;
    return (max_delta > allowed) ? allowed / max_delta : S(1.0);
}

/**
 * @brief Move every leg along its swing or stance spline
 * @param[in] dt The time since the last tick (s)
 * @param[in] idle If true, the step doesn't advance
 * @param[in] handlePhaseTransition If false, phase transitions are left to the caller
*/
void GaitEngine::performLegStep(Scalar dt, bool idle, bool handlePhaseTransition) {
    Scalar ds = idle ? S(0.0) : dt / current_phase_duration;
    std::array<Vec3, kMaxLegs> targets = legTargets(ds);
    const Scalar scale = jointSpeedScale(targets, dt);
    if (scale < 1) {
        ds *= scale;
        targets = legTargets(ds);
    }

    // Advance the splines and deactivate any that have reached their end
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        SwingSpline& spline = trajectoryGen.gaitState.swingSplines[legNum];
        LineSpline& stanceSpline = trajectoryGen.gaitState.stanceSplines[legNum];
        if (spline.active) {
            spline.s = std::min(S(1.0), spline.s + ds);
            spline.active = spline.s < 1;
        } else if (stanceSpline.active) {
            stanceSpline.s = std::min(S(1.0), stanceSpline.s + ds);
            stanceSpline.active = stanceSpline.s < 1;
        }
    }

    sendFootTargets(targets, dt);

    phase_progress = std::min(S(1.0), phase_progress + ds);
    if (!handlePhaseTransition) return;

    if (phase_progress >= 1) {
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());
        phase_progress = 0;
    }
}

/**
 * @brief Switch to a requested gait, if there is one
 * @note Only call between phases, when no leg is mid-swing. Restarts at the new gait's first phase.
*/
void GaitEngine::applyPendingGait() {
    if (!config.gaitChangeRequested) {
        return;
    }
    config.gaitChangeRequested = false;
    if (config.pendingGait == config.currentGait) {
        return;
    }
    config.currentGait = config.pendingGait;
    trajectoryGen.gaitState.config = config.getGaitConfig(config.currentGait);
    phase = 0;
    phase_progress = 0;
    leg_swinging.fill(false);
    emit(LogLevel::Info, "Gait changed to %s", config.gaitName(config.currentGait));
}

bool GaitEngine::isSwinging(int legNum) const {
    return leg_swinging[legNum] || trajectoryGen.gaitState.swingSplines[legNum].active;
}

Vec3 GaitEngine::plantedStart(int legNum) {
    return config.startPosition.at(legNum) + Vec3(0, 0, terrain_offset[legNum]);
}

/**
 * @brief Whether the foot forces can be trusted
 * @return true once the forces add up to a robot standing on its feet; false without effort data
 *         or with the robot lifted
*/
bool GaitEngine::forcesValid() {
    Scalar total = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        total += std::max(S(0.0), foot_forces[legNum].z());
    }
    return has_foot_forces && total > 3 * p.contact_force_on;
}

/**
 * @brief Follow uneven ground with the feet
 * @param[in] dt The time since the last tick (s)
 * @note A swinging foot that meets the ground in the second half of its swing stops there. One that
 *       finishes its swing without touching down keeps reaching down at seek_speed until it does.
 *       Each foot remembers its ground height (up to max_step_height either way). The body rises or
 *       falls at body_height_rate so the feet's average ground is back at nominal. Off, or without
 *       force data, the offsets ease back to zero.
*/
void GaitEngine::adaptToTerrain(Scalar dt) {
    const Vec3 nominal = config.startPosition.at(0);  // all legs share the start height
    if (!p.terrain_adaptation || !forcesValid()) {
        seeking.fill(false);
        landing_check.fill(false);
    } else {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            const Vec3& force = foot_forces[legNum];
            const Scalar sideways = force.head<2>().norm();
            const bool loaded = force.z() > p.touchdown_force;
            // Landing pushes up; catching an edge on the way across pushes back
            const bool landed = loaded && force.z() > p.touchdown_ratio * sideways;
            // Only once the foot is up, and pushed back without bearing weight, for 3 ticks
            const bool blockedFoot = leg_swinging[legNum] && phase_progress > S(0.2) && phase_progress < S(0.9) &&
                sideways > p.stumble_force && !loaded;
            stumble_ticks[legNum] = blockedFoot ? stumble_ticks[legNum] + 1 : 0;
            if (stumble_ticks[legNum] >= 3 && stumble_target[legNum] == 0) {
                stumble_target[legNum] = p.stumble_lift;
                emit(LogLevel::Debug, "Leg %d stumbled at s=%.2f (%.1f N sideways, %.1f N up)",
                    legNum + 1, static_cast<double>(phase_progress), static_cast<double>(sideways),
                    static_cast<double>(force.z()));
            }
            // Only well above where it was aiming: touching a little early is just a normal landing
            const bool aboveTarget = current_leg_positions[legNum].z() - swing_target[legNum].z() > S(0.01);
            // It has to look landed for 3 ticks in a row
            touchdown_ticks[legNum] = (leg_swinging[legNum] && phase_progress > S(0.6) && landed && aboveTarget)
                ? touchdown_ticks[legNum] + 1 : 0;
            if (touchdown_ticks[legNum] >= 3) {
                // Touched down early: plant the foot here
                touchdown_ticks[legNum] = 0;
                leg_swinging[legNum] = false;
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - nominal.z(),
                    -p.max_step_height, p.max_step_height);
                emit(LogLevel::Debug, "Leg %d touched down %.1f mm high", legNum + 1,
                    static_cast<double>(terrain_offset[legNum] * 1000));
            } else if (landing_check[legNum] && !leg_swinging[legNum]) {
                // Its swing ended unloaded: if it is still unloaded once the weight has had time to
                // come onto it, the ground is lower than expected
                if (force.z() > p.contact_force_off) {
                    landing_check[legNum] = false;
                } else if ((unloaded_time[legNum] += dt) > S(0.15)) {
                    landing_check[legNum] = false;
                    seeking[legNum] = true;
                }
            } else if (seeking[legNum] && !leg_swinging[legNum]) {
                if (force.z() > p.contact_force_on || terrain_offset[legNum] <= -p.max_step_height) {
                    seeking[legNum] = false;
                    emit(LogLevel::Debug, "Leg %d found ground %.1f mm low", legNum + 1,
                        static_cast<double>(-terrain_offset[legNum] * 1000));
                } else {
                    terrain_offset[legNum] = std::max(-p.max_step_height, terrain_offset[legNum] - p.seek_speed * dt);
                }
            }
        }
    }

    // Body height follows the average ground (or the offsets unwind when adaptation is off)
    Scalar mean = 0;
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        mean += terrain_offset[legNum] / numLegs();
    }
    const Scalar shift = std::clamp(mean, -p.body_height_rate * dt, p.body_height_rate * dt);
    ground_level += shift;  // the body rises by shift
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        terrain_offset[legNum] -= shift;
        if (!p.terrain_adaptation || !forcesValid()) {
            terrain_offset[legNum] -= std::clamp(terrain_offset[legNum], -p.body_height_rate * dt, p.body_height_rate * dt);
        }
    }
}

/**
 * @brief Whether every commanded foot is within 1 mm of its start position
*/
bool GaitEngine::feetAtStart() {
    constexpr Scalar POSITION_TOLERANCE = S(0.001);  // m
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if ((current_leg_positions[legNum] - plantedStart(legNum)).norm() > POSITION_TOLERANCE) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Step the feet back to their start positions, one gait group at a time
 * @param[in] dt The time since the last tick (s)
*/
void GaitEngine::returnToStart(Scalar dt) {
    // Standing up straight after startup comes here before the walking code has set up the gait's
    // groups, and with none the feet never stepped back in to the standing reach
    trajectoryGen.EnsureGaitConfig();

    if (phase >= trajectoryGen.gaitState.config.size()) {
        idleReturning = false;
        return_counter = 0;
        phase_progress = 0;
        return;
    }

    if (feetAtStart()) {
        idleReturning = false;
        return_counter = 0;
        phase_progress = 0;
        phase = 0;
        return;
    }

    if (phase_progress == 0) {
        current_phase_duration = p.phase_duration;
        // Swinging legs move to their start position, standing legs hold where they are
        std::array<Vec3, kMaxLegs> startPositions;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            startPositions[legNum] = plantedStart(legNum);
        }
        trajectoryGen.GenerateTrajectories(startPositions, current_leg_positions, current_leg_positions, phase);
    }

    performLegStep(dt, false, false);

    if (phase_progress >= 1) {
        return_counter++;
        phase_progress = 0;
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());

        // After all phases, finish return-to-start and handle gait change if requested
        if (return_counter >= static_cast<int>(trajectoryGen.gaitState.config.size())) {
            return_counter = 0;
            idleReturning = false;
            applyPendingGait();
        }
    }
}

/**
 * @brief Return the legs to their start positions after idle_return_delay seconds idle
 * @param[in] dt The time since the last tick (s)
 * @return true if the command is idle
*/
bool GaitEngine::handleIdleReturn(Scalar dt) {
    constexpr Scalar DEADZONE = S(0.01);

    const bool idle = (std::abs(current_motion_intent.forward) < DEADZONE &&
                       std::abs(current_motion_intent.left) < DEADZONE &&
                       std::abs(current_motion_intent.yaw) < DEADZONE);

    // Track if we've ever moved (received a non-zero command and started stepping)
    if (!idle && phase_progress > 0) {
        has_moved_from_start = true;
        return_completed = false;
    }

    if (idle) {
        idle_time += dt;
    } else {
        idle_time = 0;
    }

    if (idleReturning) {
        returnToStart(dt);
        if (!idleReturning) {
            return_completed = true;
            has_moved_from_start = false;  // don't re-trigger until the robot moves again
        }
        idle_time = 0;
    } else if (has_moved_from_start && !return_completed && idle_time > p.idle_return_delay) {
        idleReturning = true;
        phase_progress = 0;
        leg_swinging.fill(false);
        returnToStart(dt);
        idle_time = 0;
    }
    return idle;
}

/**
 * @brief Foot targets for one walking tick, without changing any state
 * @param[in] forward Forward body velocity (m/s)
 * @param[in] left Leftward body velocity (m/s)
 * @param[in] yaw Body yaw rate (rad/s)
 * @param[in] ds How far to advance the phase this tick (fraction of a phase)
 * @param[in] moveDt How much body motion to apply to the stance feet this tick (s)
 * @return The target foot position of each leg (leg frame)
*/
std::array<Vec3, kMaxLegs> GaitEngine::walkTargets(Scalar forward, Scalar left, Scalar yaw, Scalar ds, Scalar moveDt) {
    std::array<Vec3, kMaxLegs> targets;
    const Scalar s = std::min(S(1.0), phase_progress + ds);
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (leg_swinging[legNum]) {
            if (s >= 1) {
                targets[legNum] = swing_target[legNum];
                continue;
            }
            SwingSpline spline;
            trajectoryGen.genSwingSpline(spline, swing_start[legNum], swing_target[legNum],
                swing_lift[legNum] > 0 ? swing_lift[legNum] : trajectoryGen.swingLift);
            targets[legNum] = trajectoryGen.evalBezier(spline.P0, spline.P1, spline.P2, spline.P3, spline.P4, s);
            // A stumble lifts the foot higher, fading out over the last 30% so it still lands on target
            targets[legNum].z() += stumble_boost[legNum] * std::min(S(1.0), (1 - s) / S(0.3));
            targets[legNum] = limitSwingLift(legNum, targets[legNum],
                swing_start[legNum].z() + (swing_target[legNum].z() - swing_start[legNum].z()) * s);
        } else {
            const Vec3& current = current_leg_positions[legNum];
            targets[legNum] = trajectoryGen.stanceTarget(legNum, current,
                trajectoryGen.footDisplacementFrom(forward, left, yaw, legNum, current, moveDt));
            targets[legNum].z() = plantedStart(legNum).z();  // on its own ground
        }
    }
    return targets;
}

/**
 * @brief Walking, one tick
 * @param[in] dt The time since the last tick (s)
 * @note The command is re-read every tick: stance feet follow the current body velocity, and swing
 *       feet re-aim their landing target (at up to max_target_speed). The phase length also
 *       follows the command.
*/
void GaitEngine::walk(Scalar dt) {
    const bool velocityIdle = handleIdleReturn(dt);
    if (idleReturning) return;

    trajectoryGen.EnsureGaitConfig();

    // Switch gait between phases, when every foot is planted
    if (phase_progress == 0) {
        applyPendingGait();
    }
    adaptToTerrain(dt);

    // Current command, capped at what this gait can do
    Scalar forward = current_motion_intent.forward;
    Scalar left = current_motion_intent.left;
    Scalar yaw = current_motion_intent.yaw;
    Scalar footSpeed = std::hypot(forward, left) + std::abs(yaw) * trajectoryGen.maxBodyRadius();
    const Scalar maxSpeed = maxFootSpeed();
    if (footSpeed > maxSpeed) {
        const Scalar scale = maxSpeed / footSpeed;
        forward *= scale;
        left *= scale;
        yaw *= scale;
        footSpeed = maxSpeed;
    }

    // Step faster once the stride would exceed maxStride
    const size_t groups = std::max<size_t>(2, trajectoryGen.gaitState.config.size());
    current_phase_duration = p.phase_duration;
    if (footSpeed > S(1e-6)) {
        const Scalar strideLimitedDuration = 2 * config.maxStride / ((groups - 1) * footSpeed);
        current_phase_duration = std::clamp(strideLimitedDuration, p.min_phase_duration, p.phase_duration);
    }
    const Scalar stanceTime = (groups - 1) * current_phase_duration;

    // Start of a phase: the next group lifts off from where it stands
    if (phase_progress == 0) {
        stumble_target.fill(0);
        stumble_boost.fill(0);
        swing_lift.fill(0);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            leg_swinging[legNum] = trajectoryGen.gaitState.config.swingsIn(legNum, phase);
            swing_start[legNum] = current_leg_positions[legNum];
            swing_target[legNum] = current_leg_positions[legNum];
            trajectoryGen.gaitState.swingSplines[legNum].active = false;
            trajectoryGen.gaitState.stanceSplines[legNum].active = false;
        }
    }

    // Stumble lift ramps in at 0.5 m/s rather than jerking the foot up
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        stumble_boost[legNum] += std::clamp(stumble_target[legNum] - stumble_boost[legNum], -S(0.5) * dt, S(0.5) * dt);
    }

    // Swing landing targets follow the command, at a limited rate. Both ends stay inside the
    // stride circle, so every point in between does too.
    const Scalar maxTargetStep = p.max_target_speed * dt;
    const bool useMap = p.terrain_map && p.terrain_adaptation && forcesValid() &&
        footholdPlanner != nullptr && footholdPlanner->ready();
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!leg_swinging[legNum]) continue;
        Vec3 desired = trajectoryGen.swingTarget(legNum,
            trajectoryGen.footDisplacement(forward, left, yaw, legNum, stanceTime / 2));
        desired.z() = plantedStart(legNum).z();  // expect the ground where this foot last stood
        if (useMap) {
            footholdPlanner->plan(legNum, desired, swing_start[legNum], swing_lift[legNum]);
        }
        Vec3 step = desired - swing_target[legNum];
        if (step.norm() > maxTargetStep) {
            step *= maxTargetStep / step.norm();
        }
        swing_target[legNum] += step;
    }

    // Advance, slowing the whole gait if a joint would move faster than max_joint_speed
    Scalar ds = velocityIdle ? S(0.0) : dt / current_phase_duration;
    Scalar moveDt = velocityIdle ? S(0.0) : dt;
    auto targets = walkTargets(forward, left, yaw, ds, moveDt);
    const Scalar scale = jointSpeedScale(targets, dt);
    if (scale < 1) {
        ds *= scale;
        moveDt *= scale;
        targets = walkTargets(forward, left, yaw, ds, moveDt);
    }
    sendFootTargets(targets, dt);

    phase_progress = std::min(S(1.0), phase_progress + ds);
    if (phase_progress >= 1) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            // The foot is where its swing aimed: that's its ground now
            if (leg_swinging[legNum] && p.terrain_adaptation) {
                terrain_offset[legNum] = std::clamp(current_leg_positions[legNum].z() - config.startPosition.at(legNum).z(),
                    -p.max_step_height, p.max_step_height);
            }
            // A foot that has just landed takes a moment to load up: check again once the weight
            // has had time to come onto it
            if (leg_swinging[legNum] && p.terrain_adaptation && forcesValid() &&
                foot_forces[legNum].z() < p.contact_force_off) {
                landing_check[legNum] = true;
                unloaded_time[legNum] = 0;
            }
        }
        leg_swinging.fill(false);
        phase = static_cast<uint8_t>((phase + 1) % trajectoryGen.gaitState.config.size());
        phase_progress = 0;
    }
}

/**
 * @brief Foot positions for a body pose in tilt mode, and whether the legs can take it up safely
 * @param[in] pose The body pose relative to the planted feet
 * @param[out] targets The foot position of each leg (leg frame)
 * @return true if every foot stays within maxStride of neutral and every joint is within reach
*/
bool GaitEngine::tiltFeasible(const BodyPose& pose, std::array<Vec3, kMaxLegs>& targets) {
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        targets[legNum] = trajectoryGen.posedFoot(legNum, tilt_ground_feet[legNum], pose);
        const Vec3 offset = targets[legNum] - config.startPosition.at(legNum);
        if (std::hypot(offset.x(), offset.y()) > config.maxStride) {
            return false;
        }
        if (!reachable(legNum, targets[legNum])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Spread the robot's weight evenly between its planted feet
 * @param[in] dt The time since the last tick (s)
 * @note Each planted foot carrying more than its share is raised, and each carrying less lowered,
 *       at load_balance_gain per newton, with four or more feet down. Each change is stripped of
 *       any part that would raise or tilt the body (a plane fitted through the planted feet). Only
 *       while standing still; walking, the offsets are folded into each foot's terrain height.
*/
void GaitEngine::updateLoadBalance(Scalar dt) {
    constexpr Scalar MAX_RATE = S(0.02);  // m/s, fastest a foot is raised or lowered
    const Scalar maxStep = MAX_RATE * dt;

    const bool standing = std::abs(current_motion_intent.forward) < S(1e-3) &&
        std::abs(current_motion_intent.left) < S(1e-3) &&
        std::abs(current_motion_intent.yaw) < S(1e-3);  // tilt mode counts: its feet stay planted
    if (!standing && p.terrain_adaptation) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            terrain_offset[legNum] = std::clamp(terrain_offset[legNum] + foot_height_offset[legNum],
                -p.max_step_height, p.max_step_height);
            foot_height_offset[legNum] = 0;
        }
        return;
    }

    std::array<Scalar, kMaxLegs> change{};
    if (!p.load_balancing || !standing || sit_state != SitState::STANDING) {
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            change[legNum] = -foot_height_offset[legNum];
        }
    } else {
        int planted[kMaxLegs];
        int count = 0;
        Scalar total = 0;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            if (!isSwinging(legNum)) {
                planted[count++] = legNum;
                total += foot_forces[legNum].z();
            }
        }
        if (!has_foot_forces || count < 4 || total <= 0) {
            return;
        }
        const Scalar share = total / count;
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kMaxLegs, 1> raise(count);
        Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, kMaxLegs, 3> plane(count, 3);
        for (int i = 0; i < count; ++i) {
            raise(i) = p.load_balance_gain * dt * (foot_forces[planted[i]].z() - share);
            const Vec3 foot = trajectoryGen.neutralFootInBody(planted[i]);
            plane.row(i) << 1, foot.x(), foot.y();
        }
        // Remove the best-fit plane through the planted feet
        raise -= plane * plane.colPivHouseholderQr().solve(raise);
        for (int i = 0; i < count; ++i) {
            change[planted[i]] = raise(i);
        }
    }
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        foot_height_offset[legNum] = std::clamp(
            foot_height_offset[legNum] + std::clamp(change[legNum], -maxStep, maxStep),
            -p.load_balance_max, p.load_balance_max);
    }
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
 * @brief Whether every leg can put its foot at a target, e.g. a seated or standing pose
 * @param[in] target The foot position (leg frame)
*/
bool GaitEngine::everyLegReaches(const Vec3& target) const {
    for (int legNum = 0; legNum < numLegs(); ++legNum) {
        if (!reachable(legNum, target)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Lower a swinging foot until every joint of its leg is inside its range
 * @param[in] legNum The leg
 * @param[in] target The swing's foot target (leg frame)
 * @param[in] groundZ Height of the swing's ground line (lift-off to landing) under the target
 * @return target, with its height above groundZ cut back as far as needed, but no lower than groundZ
*/
Vec3 GaitEngine::limitSwingLift(int legNum, const Vec3& target, Scalar groundZ) {
    auto fits = [&](const Vec3& foot) { return reachable(legNum, trajectoryGen.posedFoot(legNum, foot, walk_level)); };
    const Scalar lift = target.z() - groundZ;
    if (lift <= 0 || fits(target)) {
        return target;
    }
    Vec3 lowered = target;
    Scalar low = 0, high = 1;  // share of the lift kept
    for (int i = 0; i < 10; ++i) {
        const Scalar mid = S(0.5) * (low + high);
        lowered.z() = groundZ + lift * mid;
        (fits(lowered) ? low : high) = mid;
    }
    lowered.z() = groundZ + lift * low;
    emitThrottled(LOG_SWING_LOWERED, 1, LogLevel::Debug, "Leg %d swing lowered by %.0f mm to stay inside its joint limits",
        legNum + 1, static_cast<double>((1 - low) * lift * 1000));
    return lowered;
}

/**
 * @brief Update the leveling correction used while walking
 * @param[in] dt The time since the last tick (s)
 * @note The correction builds up the IMU's measured roll and pitch, so the body levels out on a
 *       slope, rotating about the ground point under the body. It changes at most 0.5 rad/s and
 *       only as far as every foot can still reach. Off it unwinds; without fresh IMU data it holds.
*/
void GaitEngine::updateWalkLeveling(Scalar dt) {
    constexpr Scalar MAX_RATE = S(0.5);  // rad/s
    const Scalar maxStep = MAX_RATE * dt;

    Scalar roll = walk_level_roll;
    Scalar pitch = walk_level_pitch;
    if (!p.walk_leveling || sit_state != SitState::STANDING) {
        roll = 0;
        pitch = 0;
    } else if (imuFresh()) {
        roll -= p.walk_level_gain * dt * body_roll;
        pitch -= p.walk_level_gain * dt * body_pitch;
    }
    roll = std::clamp(roll, -p.walk_max_angle, p.walk_max_angle);
    pitch = std::clamp(pitch, -p.walk_max_angle, p.walk_max_angle);
    roll = walk_level_roll + std::clamp(roll - walk_level_roll, -maxStep, maxStep);
    pitch = walk_level_pitch + std::clamp(pitch - walk_level_pitch, -maxStep, maxStep);

    // Scale back towards level until every foot is reachable (level always is)
    const Vec3 pivot(0, 0, config.currentHeight);
    auto fits = [&](Scalar scale) {
        const BodyPose pose = TrajectoryGenerator::rotationAbout(roll * scale, pitch * scale, pivot);
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            if (!reachable(legNum, trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], pose))) {
                return false;
            }
        }
        return true;
    };
    if (!fits(1)) {
        Scalar low = 0, high = 1;
        for (int i = 0; i < 10; ++i) {
            const Scalar mid = S(0.5) * (low + high);
            (fits(mid) ? low : high) = mid;
        }
        roll *= low;
        pitch *= low;
    }
    walk_level_roll = roll;
    walk_level_pitch = pitch;
    walk_level = TrajectoryGenerator::rotationAbout(roll, pitch, pivot);
}

/**
 * @brief Tilt mode: hold the feet where they are and move the body to the requested pose
 * @param[in] dt The time since the last tick (s)
 * @param[in] exiting true once another mode is selected, to ease the body back to neutral
 * @note On entry the legs step back to their start positions first. The requested pose is rate
 *       limited, then the IMU leveling correction is added, so the body holds its angles in space
 *       when the ground beneath it tilts or turns. A pose the legs can't reach is cut back to the
 *       furthest safe pose towards it.
*/
void GaitEngine::tilt(Scalar dt, bool exiting) {
    current_motion_intent = MotionIntent{};

    if (!tilt_active) {
        // Plant every foot at its start position, so the body has the full range to move in
        if (!idleReturning && !feetAtStart()) {
            idleReturning = true;
            phase_progress = 0;
            leg_swinging.fill(false);
        }
        if (idleReturning) {
            returnToStart(dt);
            return;
        }
        // Take over from walk leveling: the feet as actually placed become tilt mode's neutral
        tilt_saved_roll = walk_level_roll;
        tilt_saved_pitch = walk_level_pitch;
        tilt_saved_feet = current_leg_positions;
        for (int legNum = 0; legNum < numLegs(); ++legNum) {
            tilt_ground_feet[legNum] = trajectoryGen.posedFoot(legNum, current_leg_positions[legNum], walk_level);
        }
        current_leg_positions = tilt_ground_feet;
        walk_level = BodyPose{};
        body_pose_request = BodyPose{};
        tilt_command = BodyPose{};
        tilt_correction = BodyPose{};
        tilt_applied = BodyPose{};
        tilt_yaw_reference = body_yaw;
        tilt_active = true;
        emit(LogLevel::Info, "Tilt mode: feet planted, body follows body_pose%s",
            p.tilt_leveling ? ", held in place by the IMU" : "");
    }

    // Move the command towards the request (or neutral when leaving) at a limited rate
    auto approach = [](Scalar& value, Scalar goal, Scalar maxStep) {
        value += std::clamp(goal - value, -maxStep, maxStep);
    };
    const BodyPose neutralPose{};
    const BodyPose& goal = exiting ? neutralPose : body_pose_request;
    const Scalar angleStep = p.tilt_angle_rate * dt;
    approach(tilt_command.roll, goal.roll, angleStep);
    approach(tilt_command.pitch, goal.pitch, angleStep);
    approach(tilt_command.yaw, goal.yaw, angleStep);
    for (int i = 0; i < 3; ++i) {
        approach(tilt_command.translation[i], goal.translation[i], p.tilt_translation_rate * dt);
    }

    // Leveling: integrate the error between the commanded and measured angles. Without fresh IMU
    // data the correction holds, and when leaving or with leveling off it unwinds.
    const BodyPose previousCorrection = tilt_correction;
    if (exiting || !p.tilt_leveling) {
        approach(tilt_correction.roll, 0, angleStep);
        approach(tilt_correction.pitch, 0, angleStep);
        approach(tilt_correction.yaw, 0, angleStep);
    } else if (imuFresh()) {
        const Scalar gain = p.tilt_level_gain * dt;
        tilt_correction.roll += gain * (tilt_command.roll - body_roll);
        tilt_correction.pitch += gain * (tilt_command.pitch - body_pitch);
        if (p.tilt_lock_yaw || controller_tilt_active) {
            tilt_correction.yaw += gain * std::remainder(tilt_command.yaw - (body_yaw - tilt_yaw_reference), 2 * kPi);
        }
    }
    // The command plus correction stays within tilt_max_angle
    tilt_correction.roll = std::clamp(tilt_correction.roll, -p.tilt_max_angle - tilt_command.roll, p.tilt_max_angle - tilt_command.roll);
    tilt_correction.pitch = std::clamp(tilt_correction.pitch, -p.tilt_max_angle - tilt_command.pitch, p.tilt_max_angle - tilt_command.pitch);
    tilt_correction.yaw = std::clamp(tilt_correction.yaw, -p.tilt_max_angle - tilt_command.yaw, p.tilt_max_angle - tilt_command.yaw);

    BodyPose desired;
    desired.roll = tilt_command.roll + tilt_correction.roll;
    desired.pitch = tilt_command.pitch + tilt_correction.pitch;
    desired.yaw = tilt_command.yaw + tilt_correction.yaw;
    desired.translation = tilt_command.translation;

    // Step from the last pose (always safe) towards the desired one, never jumping
    constexpr Scalar MAX_POSE_ANGLE_SPEED = S(2.0);   // rad/s
    constexpr Scalar MAX_POSE_SHIFT_SPEED = S(0.2);   // m/s
    const Scalar maxAngle = MAX_POSE_ANGLE_SPEED * dt;
    const Scalar maxShift = MAX_POSE_SHIFT_SPEED * dt;
    BodyPose step = tilt_applied;
    approach(step.roll, desired.roll, maxAngle);
    approach(step.pitch, desired.pitch, maxAngle);
    approach(step.yaw, desired.yaw, maxAngle);
    for (int i = 0; i < 3; ++i) {
        approach(step.translation[i], desired.translation[i], maxShift);
    }

    std::array<Vec3, kMaxLegs> targets;
    targets.fill(Vec3::Zero());
    BodyPose pose = step;
    if (!tiltFeasible(pose, targets)) {
        // Out of reach: ease yaw back towards zero instead, since turning swings the feet furthest
        BodyPose yawEased = step;
        yawEased.yaw = tilt_applied.yaw;
        approach(yawEased.yaw, 0, maxAngle);
        if (tiltFeasible(yawEased, targets)) {
            pose = yawEased;
            tilt_correction.yaw = previousCorrection.yaw;  // stop it winding up against the limit
        } else {
            // Go as far along the step as is safe, possibly nowhere
            Scalar low = 0, high = 1;
            for (int i = 0; i < 10; ++i) {
                const Scalar mid = S(0.5) * (low + high);
                BodyPose trial;
                trial.roll = tilt_applied.roll + mid * (step.roll - tilt_applied.roll);
                trial.pitch = tilt_applied.pitch + mid * (step.pitch - tilt_applied.pitch);
                trial.yaw = tilt_applied.yaw + mid * (step.yaw - tilt_applied.yaw);
                trial.translation = tilt_applied.translation + mid * (step.translation - tilt_applied.translation);
                (tiltFeasible(trial, targets) ? low : high) = mid;
            }
            pose.roll = tilt_applied.roll + low * (step.roll - tilt_applied.roll);
            pose.pitch = tilt_applied.pitch + low * (step.pitch - tilt_applied.pitch);
            pose.yaw = tilt_applied.yaw + low * (step.yaw - tilt_applied.yaw);
            pose.translation = tilt_applied.translation + low * (step.translation - tilt_applied.translation);
            tilt_correction = previousCorrection;
        }
        tiltFeasible(pose, targets);
        emitThrottled(LOG_TILT_LIMITED, 2, LogLevel::Warn, "Tilt pose limited by the legs' reach");
    }
    tilt_applied = pose;
    sendFootTargets(targets, dt);

    // Back at neutral after leaving tilt mode: hand over to walking
    const bool neutral = std::abs(pose.roll) < S(1e-4) && std::abs(pose.pitch) < S(1e-4) &&
        std::abs(pose.yaw) < S(1e-4) && pose.translation.norm() < S(1e-4);
    if (exiting && neutral) {
        tilt_active = false;
        tilt_applied = BodyPose{};
        // Hand back to walk leveling where it left off; the feet are where it had them
        walk_level_roll = tilt_saved_roll;
        walk_level_pitch = tilt_saved_pitch;
        walk_level = TrajectoryGenerator::rotationAbout(walk_level_roll, walk_level_pitch,
            Vec3(0, 0, config.currentHeight));
        current_leg_positions = tilt_saved_feet;
        phase = 0;
        phase_progress = 0;
        emit(LogLevel::Info, "Tilt mode finished, body back to neutral");
    }
}

/**
 * @brief Sitting down or standing up, one tick at a time
 * @param[in] dt The time since the last tick (s)
 * @note Sitting steps the feet out to sit_reach a gait group at a time, then lowers the body onto
 *       the ground. Standing up is the reverse. The velocity command is ignored throughout, load
 *       balancing and walk leveling unwind, and the height glide does the lowering and raising.
*/
void GaitEngine::sitTick(Scalar dt) {
    current_motion_intent = MotionIntent{};
    updateWalkLeveling(dt);

    switch (sit_state) {
        case SitState::SPREADING:
        case SitState::GATHERING:
            if (!idleReturning) {
                if (!feetAtStart()) {
                    idleReturning = true;
                    phase_progress = 0;
                    leg_swinging.fill(false);
                } else if (sit_state == SitState::SPREADING) {
                    target_height = -p.sit_height;
                    sit_state = SitState::LOWERING;
                } else {
                    sit_state = SitState::STANDING;
                    if (seatedChanged) {
                        seatedChanged(seated());
                    }
                    emit(LogLevel::Info, "Standing");
                }
            }
            if (idleReturning) {
                returnToStart(dt);
                return;
            }
            break;
        case SitState::LOWERING:
            if (config.currentHeight == target_height) {
                sit_state = SitState::SEATED;
                emit(LogLevel::Info, "Seated");
            }
            break;
        case SitState::RAISING:
            if (config.currentHeight == target_height) {
                config.setReach(stand_reach);
                sit_state = SitState::GATHERING;
            }
            break;
        case SitState::SEATED:
        case SitState::STANDING:
            break;
    }

    // Hold the feet where they are; the height glide has already moved them this tick
    sendFootTargets(current_leg_positions, dt);
}

}  // namespace gait_core
