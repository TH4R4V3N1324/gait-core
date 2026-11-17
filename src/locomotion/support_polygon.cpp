#include "gait_core/locomotion/support_polygon.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace gait_core {

namespace {

// z of (b - a) x (c - a): positive when a, b, c turn anticlockwise
Scalar cross(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

Scalar distanceToSegment(const Vec2& point, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Scalar length2 = ab.squaredNorm();
    const Scalar t = length2 > 0 ? std::clamp((point - a).dot(ab) / length2, Scalar(0), Scalar(1)) : Scalar(0);
    return (point - (a + t * ab)).norm();
}

}  // namespace

Scalar stabilityMargin(const Vec2* feet, int count, const Vec2& point) {
    count = std::clamp(count, 0, kMaxLegs);
    if (count == 0) {
        return -std::numeric_limits<Scalar>::infinity();
    }

    // Convex hull, anticlockwise (Andrew's monotone chain); points in a line or repeated are dropped
    std::array<Vec2, kMaxLegs> sorted;
    std::copy(feet, feet + count, sorted.begin());
    std::sort(sorted.begin(), sorted.begin() + count, [](const Vec2& a, const Vec2& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    std::array<Vec2, 2 * kMaxLegs> hull;
    int size = 0;
    for (int i = 0; i < count; ++i) {   // lower hull
        while (size >= 2 && cross(hull[size - 2], hull[size - 1], sorted[i]) <= 0) {
            --size;
        }
        hull[size++] = sorted[i];
    }
    for (int i = count - 2, lower = size + 1; i >= 0; --i) {   // upper hull
        while (size >= lower && cross(hull[size - 2], hull[size - 1], sorted[i]) <= 0) {
            --size;
        }
        hull[size++] = sorted[i];
    }
    if (size > 1) {
        --size;   // the last point repeats the first
    }

    if (size < 3) {
        // A foot or a line: nowhere to balance, at best on the edge
        return -distanceToSegment(point, hull[0], hull[size - 1]);
    }
    bool inside = true;
    Scalar nearestEdge = std::numeric_limits<Scalar>::infinity();
    for (int i = 0; i < size; ++i) {
        const Vec2& a = hull[i];
        const Vec2& b = hull[(i + 1) % size];
        if (cross(a, b, point) < 0) {
            inside = false;
        }
        nearestEdge = std::min(nearestEdge, distanceToSegment(point, a, b));
    }
    return inside ? nearestEdge : -nearestEdge;
}

}  // namespace gait_core
