#include "gait_core/kinematics/yaw_pitch_pitch_leg.hpp"

static_assert(gait_core::kMaxJointsPerLeg >= 3, "YawPitchPitchLeg needs GAIT_CORE_MAX_JOINTS >= 3");

namespace gait_core {

namespace {

JointAngles toJointAngles(const JointVec& v) {
    return JointAngles{v(0), v(1), v(2)};
}

JointVec toJointVec(const JointAngles& a) {
    JointVec v(3);
    v << a.coxa, a.femur, a.tibia;
    return v;
}

}  // namespace

/**
 * @brief Foot position from the joint angles, via KinematicSolver::solveFK
 * @param[in] jointAngles coxa, femur, tibia (rad)
 * @return The foot position (leg frame, m)
*/
Vec3 YawPitchPitchLeg::fk(const JointVec& jointAngles) const {
    return solver.solveFK(toJointAngles(jointAngles));
}

/**
 * @brief Joint angles that put the foot at target, via KinematicSolver::solveIK
 * @param[in] target The foot position (leg frame, m)
 * @param[in] seed Unused: the analytic solve has one solution (knee up)
 * @param[out] jointAngles coxa, femur, tibia (rad); resized to 3
 * @return false if either acos argument had to be clamped (target out of reach); jointAngles
 *         then holds the clamped solution, as solveIK always has
*/
bool YawPitchPitchLeg::ik(const Vec3& target, const JointVec& /*seed*/, JointVec& jointAngles) const {
    bool clamped = false;
    jointAngles = toJointVec(solver.solveIK(target, false, &clamped));
    return !clamped;
}

/**
 * @brief Jacobian of fk, via KinematicSolver::jacobian
 * @param[in] jointAngles coxa, femur, tibia (rad)
 * @return 3x3 d(foot position) / d(coxa, femur, tibia)
*/
LegJacobian YawPitchPitchLeg::jacobian(const JointVec& jointAngles) const {
    return solver.jacobian(toJointAngles(jointAngles));
}

}  // namespace gait_core
