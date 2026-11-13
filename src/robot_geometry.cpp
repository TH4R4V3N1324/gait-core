#include "gait_core/robot_geometry.hpp"

namespace gait_core {

RobotGeometry RobotGeometry::explorer() {
    RobotGeometry g{};
    g.coxaLength  = static_cast<Scalar>(0.063);
    g.femurLength = static_cast<Scalar>(0.100);
    g.tibiaLength = static_cast<Scalar>(0.186);

    // Coxa joint yaw (rad) and position (m), matching the URDF: the middle legs 0.11836 m out
    // to the side, the corner legs at 45 degrees from (0.11916, 0.08952) on the slimmer body
    const Scalar m  = static_cast<Scalar>(0.11836);
    const Scalar cx = static_cast<Scalar>(0.11916);
    const Scalar cy = static_cast<Scalar>(0.08952);
    g.legs[0] = {0, Vec3::Zero()};
    g.legs[1] = {-kPi / 4, Vec3(cx, -cy, 0)};       // front right
    g.legs[2] = {-kPi / 2, Vec3(0, -m, 0)};         // middle right
    g.legs[3] = {-3 * kPi / 4, Vec3(-cx, -cy, 0)};  // rear right
    g.legs[4] = {3 * kPi / 4, Vec3(-cx, cy, 0)};    // rear left
    g.legs[5] = {kPi / 2, Vec3(0, m, 0)};           // middle left
    g.legs[6] = {kPi / 4, Vec3(cx, cy, 0)};         // front left

    // Each foot straight out from its coxa (so the corner feet
    // sit at 45 degrees), 0.177 m out and 0.134 m below the hips
    g.standReach  = static_cast<Scalar>(0.177);
    g.standHeight = static_cast<Scalar>(-0.134);

    g.maxStride = static_cast<Scalar>(0.06);
    return g;
}

}  // namespace gait_core
