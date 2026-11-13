#ifndef GAIT_CORE__GAIT_ENGINE_HPP
#define GAIT_CORE__GAIT_ENGINE_HPP

#include <array>
#include <cstdint>
#include <functional>

#include "gait_core/gait_config.hpp"
#include "gait_core/kinematic_solver.hpp"
#include "gait_core/leg_odometry.hpp"
#include "gait_core/robot_geometry.hpp"
#include "gait_core/trajectory_generator.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Every setting of the gait, in SI units. Defaults are the hexapod-ros2 parameter defaults.
 * @note The ones marked "live" can be changed while running through GaitEngine::params().
*/
struct GaitParams {
    // Timing
    Scalar phase_duration = Scalar(1.0);        // s per gait phase at low speeds
    Scalar min_phase_duration = Scalar(0.7);    // s, shortest phase allowed, which sets each gait's top speed
    Scalar max_foot_speed = Scalar(1.5);        // m/s, cap on how fast a foot target can move
    Scalar max_joint_speed = Scalar(4.0);       // rad/s, phases slow down so no joint exceeds this
    Scalar idle_return_delay = Scalar(2.0);     // s idle before the legs return to their start positions
    Scalar startup_duration = Scalar(1.0);      // s for the startup move to the start positions
    Scalar max_target_speed = Scalar(0.5);      // m/s, how fast a landing target can move when the command changes
    uint8_t initial_mode = MODE_NORMAL; // mode at startup

    // Tilt mode (live)
    Scalar tilt_max_angle = Scalar(0.3);        // rad, largest roll, pitch or yaw in total
    Scalar tilt_max_translation = Scalar(0.04); // m, largest body shift along each axis
    Scalar tilt_angle_rate = Scalar(1.0);       // rad/s, how fast the commanded angles can change
    Scalar tilt_translation_rate = Scalar(0.1); // m/s, how fast the commanded shift can change
    bool tilt_leveling = true;          // hold the body's angles in space using the IMU
    bool tilt_lock_yaw = false;         // also hold the heading
    Scalar tilt_level_gain = Scalar(8.0);       // 1/s, how fast the leveling correction responds

    // Foot forces and contact (live)
    Scalar contact_force_on = Scalar(2.0);      // N, vertical force above which a foot is in contact
    Scalar contact_force_off = Scalar(1.0);     // N, below which it no longer is
    Scalar foot_force_filter = Scalar(0.05);    // s, time constant of the force low-pass filter
    Scalar blocked_force_ratio = Scalar(0.3);   // net horizontal / vertical foot force above which the robot is blocked

    // Load balancing (live)
    bool load_balancing = true;         // adjust foot heights to spread the weight evenly
    Scalar load_balance_gain = Scalar(0.0001);  // m/(N s), how fast a foot moves per newton of extra load
    Scalar load_balance_max = Scalar(0.02);     // m, largest height adjustment of a foot

    // Uneven ground (live)
    bool terrain_adaptation = true;     // stop swings on early contact and reach down for missing ground
    Scalar touchdown_force = Scalar(3.0);       // N, vertical force that ends a swing in its second half
    Scalar max_step_height = Scalar(0.04);      // m, furthest a foot's ground can be above or below nominal
    Scalar seek_speed = Scalar(0.05);           // m/s, how fast a foot that didn't find the ground reaches down
    Scalar body_height_rate = Scalar(0.02);     // m/s, how fast the body follows the feet's average ground height
    Scalar touchdown_ratio = Scalar(1.5);       // a touchdown's vertical force must be this many times its horizontal force
    Scalar stumble_force = Scalar(3.0);         // N, horizontal force on a swinging foot that means it hit something
    Scalar stumble_lift = Scalar(0.03);         // m, extra height a stumbling foot is lifted by
    bool terrain_map = false;           // use the foothold planner (a height map) when there is one

    // Swing shape (live)
    Scalar step_height = Scalar(0.06);          // m, height of the swing's middle control points
    Scalar swing_easing = Scalar(0.5);          // 0 even pace along the swing, 1 smootherstep

    // Body height and sitting (height ones live)
    Scalar height_rate = Scalar(0.03);          // m/s, how fast the body moves to a new height
    Scalar min_height = Scalar(0.06);           // m, lowest body height setHeight accepts
    Scalar max_height = Scalar(0.16);           // m, highest body height setHeight accepts
    Scalar sit_reach = Scalar(0.22);            // m, how far out the feet stand while seated
    Scalar sit_height = Scalar(0.015);          // m, feet below the hips while seated (positive)
    bool stand_on_startup = false;      // stand up as soon as startup has seated the robot

    // Leveling while walking (live)
    bool walk_leveling = true;          // hold the body level with the IMU while walking
    Scalar walk_level_gain = Scalar(2.0);       // 1/s, how fast the correction responds
    Scalar walk_max_angle = Scalar(0.15);       // rad, largest roll or pitch correction
};

enum class SitState : uint8_t {
    STANDING,
    SPREADING,  // stepping the feet out to sit_reach
    LOWERING,   // lowering the body onto the ground
    SEATED,
    RAISING,    // lifting the body back to its standing height
    GATHERING   // stepping the feet back in to the standing reach
};

/**
 * @brief Measured joint state of every leg
*/
struct JointFeedback {
    Timestamp stamp = 0;
    std::array<JointAngles, NUM_LEGS> angles{};  // rad, indexed by Leg
    std::array<bool, NUM_LEGS> present{};        // which legs' three joints were all reported
    bool hasEffort = false;                          // whether effort is filled in
    std::array<Vec3, NUM_LEGS> effort{};         // N m (coxa, femur, tibia) the servos apply
};

/**
 * @brief Body orientation from an IMU (rad), and when it was received
*/
struct ImuSample {
    Timestamp stamp = 0;
    Scalar roll = Scalar(0);
    Scalar pitch = Scalar(0);
    Scalar yaw = Scalar(0);
};

/**
 * @brief Whether a command was accepted, and a message saying what happened or why not
*/
struct CommandResult {
    bool success = false;
    char message[96] = "";
};

/**
 * @brief What one update produced
*/
struct GaitOutput {
    bool commanded = false;                           // false while waiting for joint states
    std::array<JointAngles, NUM_LEGS> joints{};  // rad, indexed by Leg
};

/**
 * @brief Optional hook for choosing footholds from a map of the ground (hexapod-ros2's height map)
*/
class FootholdPlanner {
public:
    virtual ~FootholdPlanner() = default;
    /**
     * @brief Called once per walking tick before any foothold is planned
     * @return false if the map can't be used this tick
    */
    virtual bool ready() = 0;
    /**
     * @brief Move a swinging foot's landing target and set its lift
     * @param[in] legNum The leg
     * @param[in,out] target The landing target (leg frame)
     * @param[in] swingStart Where the swing lifted off (leg frame)
     * @param[in,out] swingLift The swing's lift (m, 0 = the default step_height)
    */
    virtual void plan(int legNum, Vec3& target, const Vec3& swingStart, Scalar& swingLift) = 0;
};

/**
 * @brief The whole gait: startup, walking, sitting, tilt mode, leveling, load balancing, terrain
 *        adaptation and leg odometry, with no ROS or hardware dependency
 * @note Call update() at a steady rate, and feed it joint states, IMU orientation and velocity
 *       commands as they arrive. Every update() returns all 18 joint targets once the joint states
 *       have arrived. Optional inputs (efforts, IMU) can simply never be given: the features that
 *       need them stay off.
*/
class GaitEngine {
public:
    explicit GaitEngine(const GaitParams& params = GaitParams{},
                        const RobotGeometry& geometry = RobotGeometry::explorer());

    // ---- Inputs -----------------------------------------------------------------------------
    /**
     * @brief The commanded body velocity, already smoothed (SI, body frame)
    */
    void setVelocity(const VelocityCommand& cmd) { velocity = cmd; }
    void setImu(const ImuSample& sample);
    /**
     * @brief Measured joint state, at whatever rate it arrives
     * @return true if the leg odometry moved on (see odometry())
    */
    bool onJointStates(const JointFeedback& feedback);

    // ---- Commands ---------------------------------------------------------------------------
    CommandResult setGait(uint8_t gait);
    CommandResult setMode(uint8_t mode);
    CommandResult setHeight(Scalar height);  // feet z below the hips, negative
    CommandResult sit(bool down);
    void setBodyPose(Scalar roll, Scalar pitch, Scalar yaw, const Vec3& translation);

    // ---- Tick -------------------------------------------------------------------------------
    /**
     * @brief Advance the gait
     * @param[in] dt Time since the last update (s), already clamped by the caller
     * @param[in] now The current time
    */
    GaitOutput update(Scalar dt, Timestamp now);

    // ---- State ------------------------------------------------------------------------------
    GaitParams& params() { return p; }
    const GaitParams& params() const { return p; }
    const GaitConfig& gaitConfig() const { return config; }
    const TrajectoryGenerator& trajectory() const { return trajectoryGen; }
    const LegOdometry& odometry() const { return legOdometry; }
    SitState sitState() const { return sit_state; }
    bool initialized() const { return positions_initialized; }
    bool seated() const { return sit_state != SitState::STANDING; }
    bool blocked() const { return is_blocked; }
    const Vec3& measuredFoot(int legNum) const { return measured_leg_positions[legNum]; }
    const Vec3& footForce(int legNum) const { return foot_forces[legNum]; }
    bool footForceUpdated(int legNum) const { return foot_force_updated[legNum]; }
    Scalar maxFootSpeed() const;
    Scalar footprintHeight() const;
    Scalar floorRise() const;
    Scalar imuRoll() const { return body_roll; }
    Scalar imuPitch() const { return body_pitch; }

    // ---- Hooks ------------------------------------------------------------------------------
    std::function<void(LogLevel, const char*)> log;           // optional
    std::function<void(bool seated)> seatedChanged;         // optional: a sit or stand began or ended
    FootholdPlanner* footholdPlanner = nullptr;             // optional, not owned

private:
    GaitParams p;
    TrajectoryGenerator trajectoryGen;
    GaitConfig config;
    KinematicSolver ikSolver;
    JointLimits limits;

    VelocityCommand velocity{};
    Timestamp now_ = 0;
    GaitOutput output;

    uint8_t phase = 0;
    Scalar phase_progress = Scalar(0);    // fraction of the current phase completed, 0 to 1
    Scalar startup_progress = Scalar(0);  // fraction of the startup move completed, 0 to 1
    Scalar current_phase_duration = Scalar(1);  // s, duration chosen for the phase in progress

    // IMU
    Scalar body_roll = Scalar(0);
    Scalar body_pitch = Scalar(0);
    Scalar body_yaw = Scalar(0);
    Timestamp last_imu_time = 0;
    bool has_imu = false;

    bool has_joint_states = false;
    bool positions_initialized = false;
    bool idleReturning = false;

    struct MotionIntent {
        Scalar forward;  // m/s
        Scalar left;     // m/s
        Scalar yaw;      // rad/s, positive anticlockwise
    };
    MotionIntent current_motion_intent{};

    std::array<Vec3, NUM_LEGS> current_leg_positions;   // commanded foot positions (leg frame)
    std::array<Vec3, NUM_LEGS> measured_leg_positions;  // foot positions from joint state feedback (leg frame)
    std::array<Vec3, NUM_LEGS> startup_start_positions;
    bool startup_planned = false;

    // Walking
    std::array<Vec3, NUM_LEGS> swing_start;   // lift-off position (leg frame)
    std::array<Vec3, NUM_LEGS> swing_target;  // landing target (leg frame), rate limited
    std::array<bool, NUM_LEGS> leg_swinging{};

    // Idle return and return to start (function statics in the ROS node)
    int return_counter = 0;
    Scalar idle_time = Scalar(0);
    bool has_moved_from_start = false;
    bool return_completed = false;

    // Tilt mode
    BodyPose body_pose_request;
    BodyPose tilt_command;
    BodyPose tilt_correction;
    BodyPose tilt_applied;
    std::array<Vec3, NUM_LEGS> tilt_ground_feet;
    bool tilt_active = false;
    Scalar tilt_yaw_reference = Scalar(0);

    // Foot forces and contact
    std::array<Vec3, NUM_LEGS> foot_forces{};
    std::array<bool, NUM_LEGS> foot_contact{};
    std::array<bool, NUM_LEGS> foot_force_updated{};
    bool has_foot_forces = false;
    Timestamp last_foot_force_stamp = 0;
    std::array<Scalar, NUM_LEGS> foot_force_offset{};
    Scalar push_ratio = Scalar(0);
    bool is_blocked = false;

    // Body height and sitting
    Scalar target_height;
    SitState sit_state = SitState::SEATED;
    Scalar stand_reach;
    Scalar stand_height;

    // Uneven ground
    std::array<Scalar, NUM_LEGS> terrain_offset{};
    std::array<bool, NUM_LEGS> seeking{};
    std::array<bool, NUM_LEGS> landing_check{};
    std::array<Scalar, NUM_LEGS> unloaded_time{};
    std::array<int, NUM_LEGS> touchdown_ticks{};
    std::array<int, NUM_LEGS> stumble_ticks{};
    std::array<Scalar, NUM_LEGS> stumble_target{};
    std::array<Scalar, NUM_LEGS> stumble_boost{};
    std::array<Scalar, NUM_LEGS> swing_lift{};
    Scalar ground_level = Scalar(0);

    // Load balancing
    std::array<Scalar, NUM_LEGS> foot_height_offset{};

    // Leveling while walking
    Scalar walk_level_roll = Scalar(0);
    Scalar walk_level_pitch = Scalar(0);
    BodyPose walk_level;
    Scalar tilt_saved_roll = Scalar(0);
    Scalar tilt_saved_pitch = Scalar(0);
    std::array<Vec3, NUM_LEGS> tilt_saved_feet;

    // Leg odometry
    LegOdometry legOdometry;
    std::array<Vec2, NUM_LEGS> previous_feet;
    std::array<Scalar, NUM_LEGS> previous_feet_z{};
    std::array<Scalar, NUM_LEGS> planted_time{};
    static constexpr Scalar CLIMB_SETTLE_TIME = static_cast<Scalar>(0.1);
    static constexpr Scalar CLIMB_ANCHOR_TIME = static_cast<Scalar>(0.5);
    bool climb_anchored = false;
    Scalar odom_start_height = Scalar(0);
    std::array<bool, NUM_LEGS> previous_contact{};
    Timestamp previous_feet_stamp = 0;
    bool has_previous_feet = false;

    // Throttled log messages: when each was last sent
    std::array<Timestamp, 4> last_throttled_log{};
    bool throttled_log_sent[4] = {false, false, false, false};

    // Log a printf-style message; nothing is formatted unless log is set (and, throttled, due)
    void emit(LogLevel level, const char* format, ...);
    void emitThrottled(int key, Scalar period, LogLevel level, const char* format, ...);
    bool throttleDue(int key, Scalar period);

    void updateFootForces(const JointFeedback& feedback);
    bool updateLegOdometry(Timestamp stamp);
    Eigen::Quaternion<Scalar> imuTilt() const;
    bool imuFresh() const;
    bool feetAtStart();
    Vec3 plantedStart(int legNum);
    bool forcesValid();
    void adaptToTerrain(Scalar dt);
    void updateLoadBalance(Scalar dt);
    bool reachable(const Vec3& target);
    Vec3 limitSwingLift(int legNum, const Vec3& target, Scalar groundZ);
    void updateWalkLeveling(Scalar dt);
    bool tiltFeasible(const BodyPose& pose, std::array<Vec3, NUM_LEGS>& targets);
    void tilt(Scalar dt, bool exiting);
    void sitTick(Scalar dt);
    bool startup(Scalar dt);
    std::array<Vec3, NUM_LEGS> legTargets(Scalar ds);
    std::array<Vec3, NUM_LEGS> walkTargets(Scalar forward, Scalar left, Scalar yaw, Scalar ds, Scalar moveDt);
    void sendFootTargets(const std::array<Vec3, NUM_LEGS>& targets, Scalar dt);
    void commandFoot(const Vec3& target, int legNum);
    void applyPendingGait();
    bool isSwinging(int legNum) const;
    Scalar jointSpeedScale(const std::array<Vec3, NUM_LEGS>& targets, Scalar dt);
    void performLegStep(Scalar dt, bool idle, bool handlePhaseTransition);
    void returnToStart(Scalar dt);
    bool handleIdleReturn(Scalar dt);
    void walk(Scalar dt);
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_ENGINE_HPP
