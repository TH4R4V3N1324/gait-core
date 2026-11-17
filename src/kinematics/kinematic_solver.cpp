#include "gait_core/kinematics/kinematic_solver.hpp"

#include <algorithm>
#include <cmath>

namespace gait_core {

/**
 * @brief Inverse kinematics, calculates the joint angles based on the desired position
 * @param[in] target The desired position in 3D space (leg frame, X outward)
 * @param[in] mirrored Whether this is a mirrored leg (the left side)
 * @param[out] clamped If not null, set to whether either acos argument had to be clamped, i.e. the
 *             target was out of reach
 * @return The calculated joint angles
 * @note If the target is outside the workspace, angles are clamped to the nearest valid solution
*/
JointAngles KinematicSolver::solveIK(const Vec3& target, bool mirrored, bool* clamped) const {
    bool anyClamped = false;
    auto clamp = [&anyClamped](Scalar v) {
        if (v < Scalar(-1) || v > Scalar(1)) {
            anyClamped = true;
        }
        return std::max(Scalar(-1), std::min(Scalar(1), v));
    };

    const Scalar a1 = coxaLength;
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    const Scalar x = target.x();
    Scalar y = target.y();
    const Scalar z = target.z();

    if (mirrored) {
        y = -y;
    }

    const Scalar coxaAngle = std::atan2(y, x);
    const Scalar r1 = std::hypot(x, y) - a1;
    const Scalar r2 = z;
    const Scalar q2 = std::atan2(r2, r1);
    const Scalar r3 = std::hypot(r1, r2);
    const Scalar q1 = std::acos(clamp((a3 * a3 - a2 * a2 - r3 * r3) / (-2 * a2 * r3)));
    const Scalar femurAngle = -(q2 + q1);
    const Scalar q3 = std::acos(clamp((r3 * r3 - a2 * a2 - a3 * a3) / (-2 * a2 * a3)));
    const Scalar tibiaAngle = kPi - q3;

    if (clamped != nullptr) {
        *clamped = anyClamped;
    }
    return JointAngles{coxaAngle, femurAngle, tibiaAngle};
}

/**
 * @brief Forward kinematics, calculates the position based on the joint angles
 * @param[in] angles The joint angles
 * @return The foot position (leg frame, X outward, Z negative below the hip)
*/
Vec3 KinematicSolver::solveFK(const JointAngles& angles) const {
    const Scalar a1 = coxaLength;
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    // Elevation of each link above horizontal, matching solveIK:
    // a negative femur angle raises the femur, and tibia = 0 puts the tibia straight out from the
    // femur (positive folds it down, so the knee is at 90 degrees at tibia = pi/2)
    const Scalar femurElevation = -angles.femur;
    const Scalar tibiaElevation = femurElevation - angles.tibia;

    const Scalar horizontalReach = a1 + a2 * std::cos(femurElevation) + a3 * std::cos(tibiaElevation);
    const Scalar z = a2 * std::sin(femurElevation) + a3 * std::sin(tibiaElevation);  // negative = below the hip

    return Vec3(horizontalReach * std::cos(angles.coxa), horizontalReach * std::sin(angles.coxa), z);
}

/**
 * @brief Jacobian of the foot position with respect to the joint angles
 * @param[in] angles The joint angles
 * @return d(foot position) / d(coxa, femur, tibia), leg frame
 * @note Analytic derivative of solveFK. The ROS version used central differences with a 1e-6 rad
 *       step, which is fine in double but loses most of its digits in float.
*/
Mat3 KinematicSolver::jacobian(const JointAngles& angles) const {
    const Scalar a2 = femurLength;
    const Scalar a3 = tibiaLength;

    const Scalar e1 = -angles.femur;          // femur elevation
    const Scalar e2 = e1 - angles.tibia;      // tibia elevation
    const Scalar c = std::cos(angles.coxa);
    const Scalar s = std::sin(angles.coxa);

    const Scalar reach = coxaLength + a2 * std::cos(e1) + a3 * std::cos(e2);

    // d(e1)/d(femur) = -1, d(e2)/d(femur) = -1, d(e2)/d(tibia) = -1
    const Scalar dReachFemur = a2 * std::sin(e1) + a3 * std::sin(e2);
    const Scalar dZFemur = -(a2 * std::cos(e1) + a3 * std::cos(e2));
    const Scalar dReachTibia = a3 * std::sin(e2);
    const Scalar dZTibia = -a3 * std::cos(e2);

    Mat3 J;
    J << -reach * s, dReachFemur * c, dReachTibia * c,
          reach * c, dReachFemur * s, dReachTibia * s,
          0,         dZFemur,         dZTibia;
    return J;
}

/**
 * @brief Force the ground pushes on the foot with, from the joint torques holding the leg still
 * @param[in] angles The joint angles
 * @param[in] torques The torque each joint's servo applies (coxa, femur, tibia; N m)
 * @param[out] force The force on the foot (leg frame, N); +z means the ground is pushing up
 * @return false if the leg is too close to straight for the force to be found
 * @note In balance the servo torques cancel the ground force's torque: torques = -J^T force.
 *       The leg's own weight is ignored (about 0.1 N m on the femur).
*/
bool KinematicSolver::groundForce(const JointAngles& angles, const Vec3& torques, Vec3& force) const {
    const Mat3 Jt = jacobian(angles).transpose();
    if (std::abs(Jt.determinant()) < Scalar(1e-6)) {
        return false;
    }
    force = -Jt.inverse() * torques;
    return true;
}

}  // namespace gait_core
