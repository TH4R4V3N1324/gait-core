#ifndef GAIT_CORE__TRAJECTORY_GENERATOR_HPP
#define GAIT_CORE__TRAJECTORY_GENERATOR_HPP

#include <array>
#include <cstdint>

#include "gait_core/frame_converter.hpp"
#include "gait_core/gait_config.hpp"
#include "gait_core/robot_geometry.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

struct SwingSpline {
    Vec3 P0, P1, P2, P3, P4;
    Scalar s = 0;          // normalized progress [0,1]
    bool active = false;
};

struct LineSpline {
    Vec3 P0, P1;
    Scalar s = 0;          // normalized progress [0,1]
    bool active = false;
};

/**
 * @brief Pose of the body relative to its planted feet, used in tilt mode
*/
struct BodyPose {
    Vec3 translation = Vec3::Zero();  // m, body shift (+x forward, +y left, +z up)
    Scalar roll = 0;    // rad, positive lowers the right side
    Scalar pitch = 0;   // rad, positive lowers the nose
    Scalar yaw = 0;     // rad, positive turns anticlockwise
};

struct GaitState {
    GaitSchedule config;
    std::array<SwingSpline, NUM_LEGS> swingSplines;
    std::array<LineSpline, NUM_LEGS> stanceSplines;
};

class TrajectoryGenerator {
private:
    GaitConfig gaitConfig;
    FrameConverter converter;
public:
    explicit TrajectoryGenerator(const RobotGeometry& geometry = RobotGeometry::explorer())
        : gaitConfig(geometry), converter(geometry) {}

    GaitState gaitState;

    void GenerateTrajectories(
        const std::array<Vec3, NUM_LEGS>& swingTargets,
        const std::array<Vec3, NUM_LEGS>& stanceTargets,
        const std::array<Vec3, NUM_LEGS>& currentPositions,
        uint8_t currentPhase
    );

    Scalar legYaw(int legNum) const;
    Vec3 legToBody(int legNum, const Vec3& position) const;
    Vec3 bodyToLeg(int legNum, const Vec3& position) const;
    Vec3 posedFoot(int legNum, const Vec3& groundFoot, const BodyPose& pose) const;
    Vec3 unposedBodyPoint(const Vec3& bodyPoint, const BodyPose& pose) const;
    static BodyPose rotationAbout(Scalar roll, Scalar pitch, const Vec3& pivot);
    Vec3 neutralFootInBody(int legNum) const;
    Vec3 footDisplacement(Scalar forward, Scalar left, Scalar yaw, int legNum, Scalar duration) const;
    Vec3 footDisplacement(const VelocityCommand& cmd, int legNum, Scalar duration) const;
    Vec3 footDisplacementFrom(Scalar forward, Scalar left, Scalar yaw, int legNum, const Vec3& footPos, Scalar duration) const;
    Scalar bodyRadius(int legNum) const;
    Scalar maxBodyRadius() const;
    Vec3 swingTarget(int legNum, const Vec3& displacement) const;
    Vec3 stanceTarget(int legNum, const Vec3& currentPos, const Vec3& displacement) const;
    Vec3 clampToStride(int legNum, const Vec3& target) const;
    void EnsureGaitConfig();
    Scalar swingLift = static_cast<Scalar>(0.06);   // m, height of the swing's middle control points (peak ~0.88x this)
    Scalar swingEasing = static_cast<Scalar>(0.5);  // 0 moves along the swing at an even pace, 1 eases in and out (smootherstep)
    void genSwingSpline(SwingSpline& spline, const Vec3& start, const Vec3& end) const;
    void genSwingSpline(SwingSpline& spline, const Vec3& start, const Vec3& end, Scalar lift) const;
    Vec3 evalBezier(
        const Vec3& P0,
        const Vec3& P1,
        const Vec3& P2,
        const Vec3& P3,
        const Vec3& P4,
        Scalar s
    ) const;
};

}  // namespace gait_core

#endif  // GAIT_CORE__TRAJECTORY_GENERATOR_HPP
