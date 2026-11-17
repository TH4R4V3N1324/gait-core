#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"

#include <algorithm>
#include <cmath>

static_assert(gait_core::kMaxJointsPerLeg >= 3, "YawPitchPitchLeg needs GAIT_CORE_MAX_JOINTS >= 3");

namespace gait_core {

/**
 * @brief Foot position from the joint angles
 * @param[in] jointAngles coxa, femur, tibia (rad)
 * @return The foot position (leg frame, m: X outward, Z negative below the hip)
*/
Vec3 YawPitchPitchLeg::fk(const JointVec& jointAngles) const {
    const Scalar coxa = jointAngles(0);
    const Scalar a1 = coxaLength;
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    // Elevation of each link above horizontal: a negative femur angle raises the femur, and
    // tibia = 0 puts the tibia straight out from the femur
    const Scalar femurElevation = -jointAngles(1);
    const Scalar tibiaElevation = femurElevation - jointAngles(2);

    const Scalar horizontalReach = a1 + a2 * std::cos(femurElevation) + a3 * std::cos(tibiaElevation);
    const Scalar z = a2 * std::sin(femurElevation) + a3 * std::sin(tibiaElevation);  // negative = below the hip

    return Vec3(horizontalReach * std::cos(coxa), horizontalReach * std::sin(coxa), z);
}

/**
 * @brief Joint angles that put the foot at target
 * @param[in] target The foot position (leg frame, m)
 * @param[in] seed Unused: the analytic solve has one solution (knee up)
 * @param[out] jointAngles coxa, femur, tibia (rad); resized to 3
 * @return false if either acos argument had to be clamped (target out of reach); jointAngles
 *         then holds the nearest valid solution
*/
bool YawPitchPitchLeg::ik(const Vec3& target, const JointVec& /*seed*/, JointVec& jointAngles) const {
    bool clamped = false;
    auto clamp = [&clamped](Scalar v) {
        if (v < Scalar(-1) || v > Scalar(1)) {
            clamped = true;
        }
        return std::max(Scalar(-1), std::min(Scalar(1), v));
    };

    const Scalar a1 = coxaLength;
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    const Scalar x = target.x();
    const Scalar y = target.y();
    const Scalar z = target.z();

    const Scalar coxaAngle = std::atan2(y, x);
    const Scalar r1 = std::hypot(x, y) - a1;
    const Scalar r2 = z;
    const Scalar q2 = std::atan2(r2, r1);
    const Scalar r3 = std::hypot(r1, r2);
    const Scalar q1 = std::acos(clamp((a3 * a3 - a2 * a2 - r3 * r3) / (-2 * a2 * r3)));
    const Scalar femurAngle = -(q2 + q1);
    const Scalar q3 = std::acos(clamp((r3 * r3 - a2 * a2 - a3 * a3) / (-2 * a2 * a3)));
    const Scalar tibiaAngle = kPi - q3;

    jointAngles.resize(kNumJoints);
    jointAngles << coxaAngle, femurAngle, tibiaAngle;
    return !clamped;
}

/**
 * @brief Jacobian of fk
 * @param[in] jointAngles coxa, femur, tibia (rad)
 * @return 3x3 d(foot position) / d(coxa, femur, tibia), leg frame
 * @note Analytic. The ROS version used central differences with a 1e-6 rad step, which is fine
 *       in double but loses most of its digits in float.
*/
LegJacobian YawPitchPitchLeg::jacobian(const JointVec& jointAngles) const {
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    const Scalar e1 = -jointAngles(1);          // femur elevation
    const Scalar e2 = e1 - jointAngles(2);      // tibia elevation
    const Scalar c = std::cos(jointAngles(0));
    const Scalar s = std::sin(jointAngles(0));

    const Scalar reach = coxaLength + a2 * std::cos(e1) + a3 * std::cos(e2);

    // d(e1)/d(femur) = -1, d(e2)/d(femur) = -1, d(e2)/d(tibia) = -1
    const Scalar dReachFemur = a2 * std::sin(e1) + a3 * std::sin(e2);
    const Scalar dZFemur = -(a2 * std::cos(e1) + a3 * std::cos(e2));
    const Scalar dReachTibia = a3 * std::sin(e2);
    const Scalar dZTibia = -a3 * std::cos(e2);

    LegJacobian J(3, kNumJoints);
    J << -reach * s, dReachFemur * c, dReachTibia * c,
          reach * c, dReachFemur * s, dReachTibia * s,
          0,         dZFemur,         dZTibia;
    return J;
}

}  // namespace gait_core
