#ifndef HEXAPOD_TELEOP__GAMEPAD_TELEOP_HPP
#define HEXAPOD_TELEOP__GAMEPAD_TELEOP_HPP

#include <algorithm>
#include <cmath>
#include <optional>

namespace hexapod_teleop {

/**
 * @brief Gait engine features the face buttons switch, in button order (A, B, X, Y)
 * @note The values equal the protocol's GaitFeature (hexapod_protocol/packets.h)
 */
enum class Feature : int { TerrainAdaptation, WalkLeveling, LoadBalancing, TiltLeveling };
constexpr int NUM_FEATURES = 4;

/** @brief The GaitParams name (and ROS parameter) of a feature */
constexpr const char* featureName(Feature f) {
    switch (f) {
        case Feature::TerrainAdaptation: return "terrain_adaptation";
        case Feature::WalkLeveling: return "walk_leveling";
        case Feature::LoadBalancing: return "load_balancing";
        case Feature::TiltLeveling: return "tilt_leveling";
    }
    return "";
}

/**
 * @brief A gamepad, already normalised
 * @note Buttons are by position, named Xbox-style: a is the bottom face button whatever the
 *       gamepad (Cross on PlayStation). Apply any stick dead zone before passing it in.
 */
struct GamepadState {
    float leftX = 0, leftY = 0;     // -1..1: X positive right, Y positive forward
    float rightX = 0, rightY = 0;
    float leftTrigger = 0, rightTrigger = 0;  // 0 released .. 1
    bool dpadUp = false, dpadDown = false;
    bool a = false, b = false, x = false, y = false;
    bool leftBumper = false, rightBumper = false;
    bool start = false;             // Start, Options, Menu, +
    bool back = false;              // Back, Create, Share, View, -
    bool controllerImu = false;     // whether controllerRoll/Pitch/Yaw are valid
    float controllerRoll = 0;       // rad
    float controllerPitch = 0;      // rad
    float controllerYaw = 0;        // rad
};

/** @brief What the robot is doing, as far as the caller knows */
struct RobotState {
    bool seated = true;     // seated, or sitting down or standing up
    bool tiltMode = false;
};

struct TeleopParams {
    double powerDownHold = 2.0;   // s, hold RB this long to power down instead of sitting/standing
    double heightRepeat = 0.3;    // s between height steps while the D-pad is held
    double controllerDriveAngle = 0.35;     // rad, controller pitch/roll for full walking speed
    double controllerDriveYawAngle = 0.5;   // rad, controller yaw for full walking rotation
};

/**
 * @brief One update's worth of commands
 * @note The continuous values run -1..1; multiply by the robot's limits. Body frame: x forward,
 *       y left, z up. A robot uses the walking ones outside tilt mode and the tilt ones in it.
 */
struct TeleopOutput {
    // Walking: velocity as a fraction of the gait's top speed
    float vx = 0, vy = 0, wz = 0;
    // Tilt mode: body shift as a fraction of the largest shift, angles as a fraction of the
    // largest tilt. roll > 0 lowers the right side, pitch > 0 lowers the nose, yaw > 0 turns left.
    float shiftX = 0, shiftY = 0, shiftZ = 0;
    float roll = 0, pitch = 0, yaw = 0;
    bool controllerTilt = false;    // left bumper held with a valid controller IMU
    float controllerRoll = 0, controllerPitch = 0, controllerYaw = 0;  // rad

    // Events, at most once per press
    bool nextMode = false;                  // Start
    bool nextGait = false;                  // Back
    bool toggleFeature[NUM_FEATURES] = {};  // A, B, X, Y: switch that feature to its other state
    std::optional<bool> sit;                // RB released: true sit down, false stand up
    bool powerDown = false;                 // RB held for powerDownHold: sit, then servo power off
    int heightStep = 0;                     // D-pad, walking and standing only: +1 up, -1 down
};

/**
 * @brief The hexapod gamepad layout, shared by every stack so a gamepad drives every robot the same
 * @note Walking: left stick drives, right stick X turns. While LB is held with a controller IMU,
 *       controller pitch drives forward/back, roll strafes, and yaw turns; the orientation at
 *       LB press is neutral and the configured angles are full speed. Tilt mode: left stick shifts
 *       the body, right stick rolls and pitches it, the triggers yaw it and the D-pad raises and
 *       lowers it; LB follows the controller orientation instead. Start: next mode. Back: next gait.
 *       A, B, X, Y: terrain adaptation, walk leveling, load balancing, tilt leveling on/off.
 *       RB, when released: sit down if standing, otherwise stand up; held for powerDownHold, power
 *       down instead. D-pad while walking and standing: body height a step at a time, repeating
 *       every heightRepeat while held.
 *
 *       Call update() for every gamepad report, and regularly (say every 50 ms) while a button may
 *       be held, so the power-down hold and the height repeat fire on time. No allocation.
 */
class GamepadTeleop {
public:
    explicit GamepadTeleop(TeleopParams params = {}) : p(params) {}

    const TeleopParams& params() const { return p; }

    TeleopOutput update(const GamepadState& pad, const RobotState& robot, double nowSeconds) {
        TeleopOutput out;

        out.vx = pad.leftY;
        out.vy = -pad.leftX;
        out.wz = -pad.rightX;

        const float dpad = (pad.dpadUp ? 1.0f : 0.0f) - (pad.dpadDown ? 1.0f : 0.0f);
        out.shiftX = pad.leftY;
        out.shiftY = -pad.leftX;
        out.shiftZ = dpad;
        out.roll = pad.rightX;
        out.pitch = pad.rightY;
        out.yaw = pad.leftTrigger - pad.rightTrigger;
        out.controllerTilt = pad.leftBumper && pad.controllerImu;
        out.controllerRoll = pad.controllerRoll;
        out.controllerPitch = pad.controllerPitch;
        out.controllerYaw = pad.controllerYaw;

        const bool controller_active = pad.leftBumper && pad.controllerImu;
        if (controller_active && !robot.tiltMode) {
            if (!controller_drive_active) {
                controller_reference_roll = pad.controllerRoll;
                controller_reference_pitch = pad.controllerPitch;
                controller_reference_yaw = pad.controllerYaw;
                controller_drive_active = true;
            }
            out.vx = controllerDriveAxis(
                pad.controllerPitch - controller_reference_pitch, p.controllerDriveAngle, -1.0f);
            out.vy = controllerDriveAxis(
                pad.controllerRoll - controller_reference_roll, p.controllerDriveAngle, -1.0f);
            out.wz = controllerDriveAxis(
                std::remainder(pad.controllerYaw - controller_reference_yaw, 2.0 * 3.14159265358979323846),
                p.controllerDriveYawAngle, 1.0f);
        } else {
            controller_drive_active = false;
        }

        out.nextMode = pressed(pad.start, prev.start);
        out.nextGait = pressed(pad.back, prev.back);
        out.toggleFeature[0] = pressed(pad.a, prev.a);
        out.toggleFeature[1] = pressed(pad.b, prev.b);
        out.toggleFeature[2] = pressed(pad.x, prev.x);
        out.toggleFeature[3] = pressed(pad.y, prev.y);

        // RB: sit/stand on release, or power down once held long enough
        if (pressed(pad.rightBumper, prev.rightBumper)) {
            sitHeldSince = nowSeconds;
            powerDownSent = false;
        }
        if (pad.rightBumper && !powerDownSent && nowSeconds - sitHeldSince >= p.powerDownHold) {
            out.powerDown = true;
            powerDownSent = true;
        }
        if (!pad.rightBumper && prev.rightBumper && !powerDownSent) {
            out.sit = !robot.seated;
        }

        // D-pad: a step on a press, then every heightRepeat while held. In tilt mode it shifts
        // the body instead (shiftZ), and the engine refuses height changes while seated.
        const int direction = dpad > 0 ? 1 : dpad < 0 ? -1 : 0;
        if (direction == 0) {
            heightDirection = 0;
        } else if (direction != heightDirection || nowSeconds >= nextHeightStep) {
            heightDirection = direction;
            nextHeightStep = nowSeconds + p.heightRepeat;
            if (!robot.seated && !robot.tiltMode) out.heightStep = direction;
        }

        prev = pad;
        return out;
    }

    /** @brief Forget held buttons, e.g. when the gamepad disconnects */
    void reset() {
        prev = GamepadState{};
        powerDownSent = false;
        heightDirection = 0;
        controller_drive_active = false;
    }

private:
    static bool pressed(bool now, bool before) { return now && !before; }
    static float controllerDriveAxis(float angle, double fullScale, float sign) {
        if (fullScale <= 0.0) return 0.0f;
        return std::clamp(sign * static_cast<float>(angle / fullScale), -1.0f, 1.0f);
    }

    TeleopParams p;
    GamepadState prev;
    double sitHeldSince = 0;
    bool powerDownSent = false;
    int heightDirection = 0;
    double nextHeightStep = 0;
    bool controller_drive_active = false;
    float controller_reference_roll = 0;
    float controller_reference_pitch = 0;
    float controller_reference_yaw = 0;
};

/**
 * @brief The body height for a D-pad step (positive heights, any unit)
 * @param current Where the body is now
 * @param lastTarget The height last asked for, if any
 * @note Steps from lastTarget while the body is still within a step of it, so quick presses add up;
 *       otherwise from where the body is (the robot sat and stood, or something else moved it).
 */
inline float stepHeight(float current, std::optional<float> lastTarget, int direction, float step,
                        float minHeight, float maxHeight) {
    const float diff = lastTarget ? *lastTarget - current : 0.0f;
    const float from = (lastTarget && diff < step && diff > -step) ? *lastTarget : current;
    float target = from + static_cast<float>(direction) * step;
    if (target < minHeight) target = minHeight;
    if (target > maxHeight) target = maxHeight;
    return target;
}

}  // namespace hexapod_teleop

#endif  // HEXAPOD_TELEOP__GAMEPAD_TELEOP_HPP
