#ifndef HEXAPOD_BEHAVIOUR__BATTERY_MONITOR_HPP
#define HEXAPOD_BEHAVIOUR__BATTERY_MONITOR_HPP

#include <array>
#include <cstdint>
#include <functional>

#include "gait_core/types.hpp"

namespace hexapod_behaviour {

using gait_core::LogLevel;
using gait_core::Scalar;
using gait_core::Timestamp;

/**
 * @brief Battery level, worst last
*/
enum class BatteryLevel : uint8_t { Ok, Warning, Critical };

/**
 * @brief Battery behaviour settings. Defaults are the hexapod-ros2 behaviour_manager defaults (a 3S pack).
*/
struct BatteryParams {
    Scalar warning_voltage = Scalar(10.5);   // V, below this the battery is low
    Scalar critical_voltage = Scalar(9.9);   // V, below this the robot sits down
    Scalar hysteresis = Scalar(0.2);         // V, how far above a threshold the voltage must recover to clear it
    Scalar filter_time = Scalar(5.0);        // s, time constant of the voltage filter, so walking doesn't trip it
    Scalar stale_timeout = Scalar(2.0);      // s, a monitor with no reading for this long is ignored
    bool auto_sit = true;                    // sit down at critical
};

/**
 * @brief One power monitor's latest reading
*/
struct PowerReading {
    Scalar voltage = 0;      // V, filtered
    Scalar current = 0;      // A, latest (negative while discharging)
    Timestamp stamp = 0;     // when the latest reading arrived
    bool received = false;
};

/**
 * @brief What the robot should do after an update
*/
struct BatteryStatus {
    BatteryLevel level = BatteryLevel::Ok;
    bool hasData = false;    // at least one monitor has a recent reading
    Scalar voltage = 0;      // V, lowest recent monitor's filtered voltage (only valid with hasData)
    int staleMonitors = 0;   // monitors with no recent reading
    bool changed = false;    // the level changed this update
    bool worsened = false;   // the level got worse this update: alert the operator (rumble, beep, LED)
    bool sit = false;        // sit the robot down; call sitRequested() once the request has gone out
};

/**
 * @brief Watches the battery through one or more power monitors and decides when to warn and sit
 * @note The lowest monitor's voltage, low-pass filtered, sets the level, with hysteresis so it
 *       doesn't flicker at a threshold. Without recent readings the level holds: a dropped
 *       connection is no reason to sit down or to clear a warning. Critical asks for the robot to
 *       sit once; it never stands it back up, since standing it up while critical is the
 *       operator's override. A robot without a power monitor just never calls onReading.
*/
class BatteryMonitor {
public:
    static constexpr int MAX_MONITORS = 4;

    /**
     * @param[in] monitors How many power monitors the robot has (1 to MAX_MONITORS)
     * @param[in] params The settings
    */
    explicit BatteryMonitor(int monitors = 1, const BatteryParams& params = {});

    void onReading(int monitor, Scalar voltage, Scalar current, Timestamp stamp);
    BatteryStatus update(Timestamp now);
    void sitRequested();

    BatteryParams& params() { return p; }
    const BatteryParams& params() const { return p; }
    BatteryLevel level() const { return level_; }
    int monitors() const { return monitor_count; }
    const PowerReading& reading(int monitor) const { return readings.at(monitor); }
    bool stale(int monitor, Timestamp now) const;

    // Hook (optional): log messages when the level changes
    std::function<void(LogLevel, const char*)> log;

private:
    BatteryLevel levelFor(Scalar voltage) const;
    void levelChanged(BatteryLevel previous, Scalar voltage);

    BatteryParams p;
    int monitor_count;
    std::array<PowerReading, MAX_MONITORS> readings{};
    BatteryLevel level_ = BatteryLevel::Ok;
    bool sit_pending = false;   // critical, and the sit request hasn't gone out yet
};

}  // namespace hexapod_behaviour

#endif  // HEXAPOD_BEHAVIOUR__BATTERY_MONITOR_HPP
