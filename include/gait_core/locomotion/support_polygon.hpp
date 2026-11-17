#ifndef GAIT_CORE__SUPPORT_POLYGON_HPP
#define GAIT_CORE__SUPPORT_POLYGON_HPP

#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief How far a point is inside the support polygon of some planted feet
 * @param[in] feet The planted feet's ground positions (x, y), in any order; points inside the
 *            convex hull and repeats are fine
 * @param[in] count How many feet, 0..kMaxLegs (more are ignored)
 * @param[in] point The point to judge, e.g. the centre of mass projected onto the ground
 * @return m, the distance from point to the nearest edge of the feet's convex hull: positive
 *         inside, zero on an edge, negative outside. With fewer than three feet not in a line
 *         there is no area to stand in, so the result is never positive: minus the distance to the
 *         line, the one foot, or -infinity with no feet.
 * @note Static stability only: a robot holding still is stable when its centre of mass is above
 *       the polygon, and the margin is how far it could be pushed before it tips.
*/
Scalar stabilityMargin(const Vec2* feet, int count, const Vec2& point);

}  // namespace gait_core

#endif  // GAIT_CORE__SUPPORT_POLYGON_HPP
