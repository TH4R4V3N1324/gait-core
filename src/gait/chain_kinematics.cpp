#include "gait_core/chain_kinematics.hpp"

#include <algorithm>
#include <array>

namespace gait_core {

ChainKinematics::ChainKinematics(const LegDefinition& leg) : leg_(leg) {
    leg_.numJoints = std::clamp(leg_.numJoints, 0, kMaxJointsPerLeg);
    for (int j = 0; j < leg_.numJoints; ++j) {
        leg_.joints[j].axis.normalize();
    }
}

/**
 * @brief Foot position from the joint angles
 * @param[in] q One angle per joint, body to foot (rad)
 * @return The foot position in the leg frame (m)
 * @note Walks the chain: step to each joint's origin and turn about its axis, both given at zero
 *       angles, so each is first turned by every joint before it. Then step out to the foot.
*/
Vec3 ChainKinematics::fk(const JointVec& q) const {
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    for (int j = 0; j < leg_.numJoints; ++j) {
        const JointDefinition& joint = leg_.joints[j];
        position += rotation * joint.origin;
        rotation = rotation * AngleAxis(q(j), joint.axis).toRotationMatrix();
    }
    return position + rotation * leg_.foot;
}

/**
 * @brief Joint angles that put the foot at target, by damped least squares from the seed
 * @param[in] target The foot position in the leg frame (m)
 * @param[in] seed Where to start, normally the current angles; zeros if it's the wrong size
 * @param[out] q The angles, always within the joint limits
 * @return true if the foot is within 1e-5 m of target. false if it isn't after MAX_ITERATIONS:
 *         out of reach, or occasionally a seed far from any solution. q is then the closest pose
 *         found, never further from target than the seed.
 * @note Each step solves J dq = error as (J^T (J J^T + damping^2 I)^-1) error, which stays well
 *       behaved near a straight leg where J loses rank. J J^T is 3x3 for any number of joints.
 *       Each step aims at most MAX_STEP closer: a target far out of reach would otherwise ask for
 *       joint moves of several radians, overshoot, and end up anywhere.
*/
bool ChainKinematics::ik(const Vec3& target, const JointVec& seed, JointVec& q) const {
    constexpr int MAX_ITERATIONS = 20;          // bounded, so the time per tick is too
    constexpr Scalar TOLERANCE = Scalar(1e-5);  // m
    constexpr Scalar DAMPING = Scalar(0.01);
    constexpr Scalar MAX_STEP = Scalar(0.05);   // m, how far one step aims to move the foot

    if (seed.size() == numJoints()) {
        q = seed;
    } else {
        q = JointVec::Zero(numJoints());
    }

    JointVec best = q;
    Scalar bestError = (target - fk(q)).norm();
    for (int i = 0; i < MAX_ITERATIONS && bestError >= TOLERANCE; ++i) {
        Vec3 step = target - fk(q);
        const Scalar distance = step.norm();
        if (distance > MAX_STEP) {
            step *= MAX_STEP / distance;
        }
        const LegJacobian J = jacobian(q);
        const Mat3 A = J * J.transpose() + DAMPING * DAMPING * Mat3::Identity();
        q += J.transpose() * A.ldlt().solve(step);
        for (int j = 0; j < leg_.numJoints; ++j) {
            q(j) = std::clamp(q(j), leg_.joints[j].lower, leg_.joints[j].upper);
        }
        const Scalar error = (target - fk(q)).norm();
        if (error < bestError) {
            best = q;
            bestError = error;
        }
    }
    q = best;
    return bestError < TOLERANCE;
}

/**
 * @brief How the foot moves with each joint
 * @param[in] q One angle per joint, body to foot (rad)
 * @return 3 x numJoints: column j is the foot's velocity (m/s) for joint j turning at 1 rad/s,
 *         which for a revolute joint is its axis crossed with the line from the joint to the foot
*/
LegJacobian ChainKinematics::jacobian(const JointVec& q) const {
    Mat3 rotation = Mat3::Identity();
    Vec3 position = Vec3::Zero();
    std::array<Vec3, kMaxJointsPerLeg> jointPos;
    std::array<Vec3, kMaxJointsPerLeg> jointAxis;
    for (int j = 0; j < leg_.numJoints; ++j) {
        const JointDefinition& joint = leg_.joints[j];
        position += rotation * joint.origin;
        jointPos[j] = position;                 // where joint j is
        jointAxis[j] = rotation * joint.axis;   // which way it points (turning about it doesn't move it)
        rotation = rotation * AngleAxis(q(j), joint.axis).toRotationMatrix();
    }
    const Vec3 foot = position + rotation * leg_.foot;

    LegJacobian J(3, leg_.numJoints);
    for (int j = 0; j < leg_.numJoints; ++j) {
        J.col(j) = jointAxis[j].cross(foot - jointPos[j]);
    }
    return J;
}

}  // namespace gait_core
