#include "gait_core/robot_definition.hpp"

namespace gait_core {

static_assert(kMaxLegs >= NUM_LEGS, "explorer() has 6 legs: GAIT_CORE_MAX_LEGS must be at least 6");

namespace {

/**
 * @brief Helper to make a revolute joint definition
 * @param origin The joint's position relative to the previous joint (or the leg frame origin),
 *        at zero angles (m)
 * @param axis The joint's unit axis, in the leg frame at zero angles
 * @param lower The joint's lower limit (rad)
 * @param upper The joint's upper limit (rad)
*/
JointDefinition revolute(const Vec3& origin, const Vec3& axis, double lower, double upper) {
    return {origin, axis, static_cast<Scalar>(lower), static_cast<Scalar>(upper)};
}

/**
 * @brief Helper to make a Vec3 from doubles
 * @param x The x component
 * @param y The y component
 * @param z The z component
*/
Vec3 v(double x, double y, double z) {
    return Vec3(static_cast<Scalar>(x), static_cast<Scalar>(y), static_cast<Scalar>(z));
}

}  // namespace

RobotDefinition RobotDefinition::explorer() {
    RobotDefinition g{};
    g.numLegs = NUM_LEGS;

    // The femur and tibia ranges match femur_lower/upper and tibia_lower/upper in the URDF (from the
    // CAD meshes); the coxa keeps +-1.4 rad until its range is worked out
    const JointDefinition coxa = revolute(v(0, 0, 0), v(0, 0, 1), -1.4, 1.4);
    const JointDefinition femur = revolute(v(0.063, 0, 0), v(0, 1, 0), -1.6, 1.4);    // + lowers the foot
    const JointDefinition tibia = revolute(v(0.100, 0, 0), v(0, 1, 0), -0.78, 2.63);  // + folds the knee

    // Coxa joint yaw (rad) and position (m), matching the URDF: the middle legs 0.11836 m out
    // to the side, the corner legs at 45 degrees from (0.11916, 0.08952) on the slimmer body
    const Scalar m  = static_cast<Scalar>(0.11836);
    const Scalar cx = static_cast<Scalar>(0.11916);
    const Scalar cy = static_cast<Scalar>(0.08952);
    const Vec3 foot_offset = v(0.186, 0, 0);

    auto leg = [&](Scalar yaw, const Vec3& coxaPosition) {
        LegDefinition def{{yaw, coxaPosition}, 3, {coxa, femur, tibia}, foot_offset};
        def.solver = LegSolver::Hexapod3Dof;  // the analytic solver matches the URDF
        return def;
    };

    g.legs[LEG_FRONT_RIGHT]  = leg(-kPi / 4, Vec3(cx, -cy, 0));
    g.legs[LEG_MIDDLE_RIGHT] = leg(-kPi / 2, Vec3(0, -m, 0));
    g.legs[LEG_REAR_RIGHT]   = leg(-3 * kPi / 4, Vec3(-cx, -cy, 0));
    g.legs[LEG_REAR_LEFT]    = leg(3 * kPi / 4, Vec3(-cx, cy, 0));
    g.legs[LEG_MIDDLE_LEFT]  = leg(kPi / 2, Vec3(0, m, 0));
    g.legs[LEG_FRONT_LEFT]   = leg(kPi / 4, Vec3(cx, cy, 0));

    // Each foot straight out from its coxa (so the corner feet
    // sit at 45 degrees), 0.177 m out and 0.134 m below the hips
    g.standReach  = static_cast<Scalar>(0.177);
    g.standHeight = static_cast<Scalar>(-0.134);

    g.maxStride = static_cast<Scalar>(0.06);
    
    return g;
}

}  // namespace gait_core
