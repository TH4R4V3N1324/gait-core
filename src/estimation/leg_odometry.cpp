#include "gait_core/estimation/leg_odometry.hpp"

#include <cmath>

namespace gait_core {

/**
 * @brief Find the body motion that best explains how planted feet moved relative to the body
 * @param[in] before Planted foot positions at the start of the update (body frame)
 * @param[in] after The same feet at the end of the update (body frame), in the same order
 * @param[in] count How many feet
 * @param[out] motion The body's motion, in the body frame at the start of the update
 * @return true if there were enough feet (at least 2) to estimate the motion, false otherwise
 * @note A least-squares rigid fit: the feet are fixed in the world, so after = R(-dyaw) * (before - t).
*/
bool LegOdometry::estimateMotion(const Vec2* before, const Vec2* after, size_t count, BodyMotion& motion) {
    if (count < 2) {
        return false;
    }

    Vec2 meanBefore = Vec2::Zero();
    Vec2 meanAfter = Vec2::Zero();
    for (size_t i = 0; i < count; ++i) {
        meanBefore += before[i];
        meanAfter += after[i];
    }
    meanBefore /= static_cast<Scalar>(count);
    meanAfter /= static_cast<Scalar>(count);

    // Rotation that best maps the feet before onto the feet after
    Scalar sumCos = 0;
    Scalar sumSin = 0;
    for (size_t i = 0; i < count; ++i) {
        const Vec2 a = before[i] - meanBefore;
        const Vec2 b = after[i] - meanAfter;
        sumCos += a.x() * b.x() + a.y() * b.y();
        sumSin += a.x() * b.y() - a.y() * b.x();
    }
    const Scalar footRotation = std::atan2(sumSin, sumCos);

    // The feet appear to rotate the opposite way to the body
    motion.dyaw = -footRotation;
    const Rotation2D bodyRotation(motion.dyaw);
    const Vec2 translation = meanBefore - bodyRotation * meanAfter;
    motion.dx = translation.x();
    motion.dy = translation.y();
    return true;
}

/**
 * @brief Find how far the body rose from how planted feet moved up or down relative to it
 * @param[in] before Planted foot heights at the start of the update (level body frame, m)
 * @param[in] after The same feet's heights at the end of the update, in the same order
 * @param[in] count How many feet
 * @param[out] climb How far the body rose (m)
 * @return true if there were enough feet (at least 2) to estimate the climb, false otherwise
 * @note The feet are fixed in the world, so they drop relative to the body as far as it rises.
 *       The heights must be in a frame levelled by the body's real tilt, or rocking reads as climbing.
*/
bool LegOdometry::estimateClimb(const Scalar* before, const Scalar* after, size_t count, Scalar& climb) {
    if (count < 2) {
        return false;
    }
    Scalar sum = 0;
    for (size_t i = 0; i < count; ++i) {
        sum += before[i] - after[i];
    }
    climb = sum / static_cast<Scalar>(count);
    return true;
}

/**
 * @brief Add one update's body motion to the pose
 * @param[in] motion The body's motion, in the body frame at the start of the update
*/
void LegOdometry::integrate(const BodyMotion& motion) {
    x_ += motion.dx * std::cos(yaw_) - motion.dy * std::sin(yaw_);
    y_ += motion.dx * std::sin(yaw_) + motion.dy * std::cos(yaw_);
    yaw_ = std::remainder(yaw_ + motion.dyaw, 2 * kPi);
}

/**
 * @brief Add one update's climb to the height
 * @param[in] climb How far the body rose (m)
*/
void LegOdometry::integrateClimb(Scalar climb) {
    z_ += climb;
}

/**
 * @brief Put the pose and height back at the origin
*/
void LegOdometry::reset() {
    x_ = 0;
    y_ = 0;
    yaw_ = 0;
    z_ = 0;
}

}  // namespace gait_core
