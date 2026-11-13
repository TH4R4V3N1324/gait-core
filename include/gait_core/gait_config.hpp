#ifndef GAIT_CORE__GAIT_CONFIG_HPP
#define GAIT_CORE__GAIT_CONFIG_HPP

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "gait_core/robot_geometry.hpp"
#include "gait_core/types.hpp"

namespace gait_core {

static constexpr int MAX_RESOLUTION = 50;
// Explorer defaults; the stride actually used comes from RobotGeometry::maxStride
static constexpr Scalar MAX_STRIDE_LENGTH = static_cast<Scalar>(0.06);  // m
static constexpr Scalar LIFT_HEIGHT = static_cast<Scalar>(0.05);        // m

struct LocomotionOption {
    std::string name;
    uint8_t id;
};

enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    NUM_MODES
};

class GaitConfig {
private:
    const std::vector<LocomotionOption> gaits = {
        {"tripod", GAIT_TRIPOD},
        {"ripple", GAIT_RIPPLE},
        {"wave", GAIT_WAVE}
    };
    const std::vector<LocomotionOption> modes = {
        {"normal", MODE_NORMAL},
        {"strafe", MODE_STRAFE},
        {"tilt", MODE_TILT}
    };
public:
    /**
     * @brief Set up the neutral stance from the robot's geometry
     * @param[in] geometry The robot whose stance to use
    */
    explicit GaitConfig(const RobotGeometry& geometry = RobotGeometry::explorer());

    /**
     * @brief Get the supported gaits
     * @return The list of supported gaits
    */
    const std::vector<LocomotionOption>& getGaits() const { return gaits; }
    /**
     * @brief Get the supported modes
     * @return The list of supported modes
    */
    const std::vector<LocomotionOption>& getModes() const { return modes; }

    Scalar currentHeight;
    Scalar maxStride;
    Vec3 homePos;
    Vec3 startPos;
    std::array<Vec3, MAX_LEGS + 1> startPosition;   // indexed by leg number, [0] unused
    Gait currentGait = GAIT_TRIPOD;
    Mode currentMode = MODE_NORMAL;
    bool gaitChangeRequested = false;
    Gait pendingGait = GAIT_TRIPOD;
    std::vector<std::vector<int>> getGaitConfig(Gait gait) const;
    void cycleGait();
    void setGait(Gait gait);
    void cycleMode();
    void setMode(Mode mode);
    void setHeight(Scalar height);
    void setReach(Scalar reach);
};

}  // namespace gait_core

#endif  // GAIT_CORE__GAIT_CONFIG_HPP
