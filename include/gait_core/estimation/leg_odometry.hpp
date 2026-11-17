#ifndef GAIT_CORE__LEG_ODOMETRY_HPP
#define GAIT_CORE__LEG_ODOMETRY_HPP

#include <cstddef>
#include <vector>

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Body motion over one update, in the body frame at the start of the update
*/
struct BodyMotion {
    Scalar dx;     // m, forward
    Scalar dy;     // m, left
    Scalar dyaw;   // rad, anticlockwise
};

/**
 * @brief Estimates body motion from planted feet and integrates it into a 2D pose and a height
 * @note Feet on the ground are assumed not to slip, so the body motion is the rigid
 *       transform that best explains how those feet moved relative to the body.
*/
class LegOdometry {
public:
    static bool estimateMotion(const Vec2* before, const Vec2* after, size_t count, BodyMotion& motion);
    static bool estimateClimb(const Scalar* before, const Scalar* after, size_t count, Scalar& climb);

    // Convenience overloads; the pointer versions above allocate nothing
    static bool estimateMotion(const std::vector<Vec2>& before, const std::vector<Vec2>& after, BodyMotion& motion) {
        return before.size() == after.size() && estimateMotion(before.data(), after.data(), before.size(), motion);
    }
    static bool estimateClimb(const std::vector<Scalar>& before, const std::vector<Scalar>& after, Scalar& climb) {
        return before.size() == after.size() && estimateClimb(before.data(), after.data(), before.size(), climb);
    }
    void integrate(const BodyMotion& motion);
    void integrateClimb(Scalar climb);
    void reset();

    Scalar x() const { return x_; }
    Scalar y() const { return y_; }
    Scalar yaw() const { return yaw_; }
    Scalar z() const { return z_; }

private:
    Scalar x_ = 0;
    Scalar y_ = 0;
    Scalar yaw_ = 0;
    Scalar z_ = 0;   // m, how far the body has risen since the start
};

}  // namespace gait_core

#endif  // GAIT_CORE__LEG_ODOMETRY_HPP
