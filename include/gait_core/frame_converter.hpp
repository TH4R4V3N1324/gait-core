#ifndef GAIT_CORE__FRAME_CONVERTER_HPP
#define GAIT_CORE__FRAME_CONVERTER_HPP

#include <array>
#include <cmath>

#include "gait_core/robot_geometry.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

/**
 * @brief Converts positions between each leg's frame and the body frame
 * @note The body frame follows REP 103 (+x forward, +y left, +z up) and matches the URDF.
 *       Each leg frame has +x pointing out along the leg from its coxa joint.
*/
class FrameConverter {
public:
    std::array<LegConfig, NUM_LEGS> legConfigs;   // indexed by Leg

    /**
     * @brief Initialise the mounting configuration of each leg
     * @param[in] geometry The robot whose legs to use
    */
    explicit FrameConverter(const RobotGeometry& geometry = RobotGeometry::explorer())
        : legConfigs(geometry.legs) {}

    /**
     * @brief Convert position from leg frame to body frame
     * @param[in] position Position in leg frame (m)
     * @param[in] leg_num The leg
     * @return Position in body frame (m)
    */
    Vec3 legToBodyFrame(const Vec3& position, int leg_num) const {
        const auto& config = legConfigs.at(leg_num);
        const Scalar c = std::cos(config.yaw);
        const Scalar s = std::sin(config.yaw);

        // Rotate from leg frame to body frame (positive rotation), then translate by the coxa offset
        return Vec3(position.x() * c - position.y() * s,
                    position.x() * s + position.y() * c,
                    position.z()) + config.translation_offset;
    }

    /**
     * @brief Convert position from body frame to leg frame
     * @param[in] position Position in body frame (m)
     * @param[in] leg_num The leg
     * @return Position in leg frame (m)
    */
    Vec3 bodyToLegFrame(const Vec3& position, int leg_num) const {
        const auto& config = legConfigs.at(leg_num);
        const Scalar c = std::cos(config.yaw);
        const Scalar s = std::sin(config.yaw);

        // Remove the coxa offset, then rotate from body frame to leg frame (negative rotation)
        const Vec3 t = position - config.translation_offset;
        return Vec3(t.x() * c + t.y() * s,
                    -t.x() * s + t.y() * c,
                    t.z());
    }

    /**
     * @brief Get the leg configuration for a specific leg
     * @param[in] leg_num The leg
     * @return Reference to leg configuration
    */
    const LegConfig& getLegConfig(int leg_num) const {
        return legConfigs.at(leg_num);
    }

    /**
     * @brief Get coxa attachment point in body frame
     * @param[in] leg_num The leg
     * @return Position of coxa joint in body frame (m)
    */
    Vec3 getCoxaPosition(int leg_num) const {
        return legConfigs.at(leg_num).translation_offset;
    }
};

}  // namespace gait_core

#endif  // GAIT_CORE__FRAME_CONVERTER_HPP
