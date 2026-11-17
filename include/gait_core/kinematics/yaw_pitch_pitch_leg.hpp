#ifndef GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP
#define GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP

#include "gait_core/kinematics/leg_kinematics.hpp"
#include "gait_core/robot/leg_definition.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Exact, analytic kinematics for a leg of a yaw joint then two pitch joints, all in a line
 *        (a coxa about z, then femur and tibia about y, as on explorer and most hexapods)
 * @note Joints are (coxa, femur, tibia). A negative femur angle raises the femur; tibia = 0 puts
 *       the tibia straight out from the femur, and positive folds it down (90 degrees at pi/2).
 *       The engine uses it for a leg whose definition asks for LegSolver::YawPitchPitch and has
 *       that shape; any other leg gets ChainKinematics.
*/
class YawPitchPitchLeg : public LegKinematics {
public:
    /**
     * @brief Take the link lengths from the leg's chain
     * @param[in] leg A yaw-pitch-pitch leg: joints[1].origin is the coxa link, joints[2].origin the
     *            femur, foot the tibia (the straight line from the knee axis to the foot)
    */
    explicit YawPitchPitchLeg(const LegDefinition& leg)
        : coxaLength(leg.joints[1].origin.norm()),
          femurLength(leg.joints[2].origin.norm()),
          tibiaLength(leg.foot.norm()) {}

    static constexpr int kNumJoints = 3;  // coxa, femur, tibia

    int numJoints() const override { return kNumJoints; }
    Vec3 fk(const JointVec& jointAngles) const override;
    bool ik(const Vec3& target, const JointVec& seed, JointVec& jointAngles) const override;
    LegJacobian jacobian(const JointVec& jointAngles) const override;

private:
    Scalar coxaLength;
    Scalar femurLength;
    Scalar tibiaLength;
};

}  // namespace gait_core

#endif  // GAIT_CORE__YAW_PITCH_PITCH_LEG_HPP
