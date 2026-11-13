#include "hexapod_behaviour/battery_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hexapod_behaviour {

using gait_core::toSeconds;

BatteryMonitor::BatteryMonitor(int monitors, const BatteryParams& params)
    : p(params), monitor_count(std::clamp(monitors, 1, MAX_MONITORS)) {}

/**
 * @brief Take a power monitor's reading, filtering its voltage with a low-pass of time constant filter_time
 * @param[in] monitor Which monitor (0 to monitors() - 1); others are ignored
 * @param[in] voltage Its voltage (V); a non-finite reading is ignored
 * @param[in] current Its current (A, negative while discharging)
 * @param[in] stamp When the reading arrived
*/
void BatteryMonitor::onReading(int monitor, Scalar voltage, Scalar current, Timestamp stamp) {
    if (monitor < 0 || monitor >= monitor_count || !std::isfinite(voltage)) {
        return;
    }
    PowerReading& reading = readings[monitor];
    if (!reading.received) {
        reading.voltage = voltage;
    } else {
        const Scalar dt = std::max(Scalar(0), static_cast<Scalar>(toSeconds(stamp - reading.stamp)));
        const Scalar alpha = p.filter_time > 0 ? 1 - std::exp(-dt / p.filter_time) : Scalar(1);
        reading.voltage += alpha * (voltage - reading.voltage);
    }
    reading.current = current;
    reading.stamp = stamp;
    reading.received = true;
}

/**
 * @brief Whether a monitor has gone without a reading for longer than stale_timeout
*/
bool BatteryMonitor::stale(int monitor, Timestamp now) const {
    const PowerReading& reading = readings.at(monitor);
    return !reading.received || toSeconds(now - reading.stamp) > static_cast<double>(p.stale_timeout);
}

/**
 * @brief Work out the battery level and what to do about it
 * @param[in] now The current time
 * @return The level, the voltage, and whether to alert or sit
*/
BatteryStatus BatteryMonitor::update(Timestamp now) {
    BatteryStatus status;
    for (int i = 0; i < monitor_count; ++i) {
        if (stale(i, now)) {
            ++status.staleMonitors;
        } else if (!status.hasData || readings[i].voltage < status.voltage) {
            status.voltage = readings[i].voltage;
            status.hasData = true;
        }
    }

    // Without data the level holds
    if (status.hasData) {
        const BatteryLevel previous = level_;
        level_ = levelFor(status.voltage);
        if (level_ != previous) {
            status.changed = true;
            status.worsened = level_ > previous;
            levelChanged(previous, status.voltage);
        }
    }
    status.level = level_;
    status.sit = level_ == BatteryLevel::Critical && sit_pending;
    return status;
}

/**
 * @brief Tell the monitor the sit request has gone out, so it isn't asked for again
*/
void BatteryMonitor::sitRequested() {
    sit_pending = false;
}

/**
 * @brief The battery level for a voltage, with hysteresis so it doesn't flicker at a threshold
 * @param[in] voltage The lowest monitor's filtered voltage (V)
 * @return The new level; recovering needs hysteresis above the threshold that was crossed
*/
BatteryLevel BatteryMonitor::levelFor(Scalar voltage) const {
    if (voltage < p.critical_voltage ||
        (level_ == BatteryLevel::Critical && voltage < p.critical_voltage + p.hysteresis)) {
        return BatteryLevel::Critical;
    }
    if (voltage < p.warning_voltage ||
        (level_ != BatteryLevel::Ok && voltage < p.warning_voltage + p.hysteresis)) {
        return BatteryLevel::Warning;
    }
    return BatteryLevel::Ok;
}

/**
 * @brief Log a change of battery level, and ask to sit at critical
 * @param[in] previous The level before the change
 * @param[in] voltage The voltage that caused it (V)
*/
void BatteryMonitor::levelChanged(BatteryLevel previous, Scalar voltage) {
    char message[64];
    LogLevel severity = LogLevel::Info;
    const double v = static_cast<double>(voltage);
    switch (level_) {
        case BatteryLevel::Ok:
            std::snprintf(message, sizeof(message), "Battery OK (%.2f V)", v);
            break;
        case BatteryLevel::Warning:
            severity = LogLevel::Warn;
            if (previous == BatteryLevel::Ok) {
                std::snprintf(message, sizeof(message), "Battery low (%.2f V)", v);
            } else {
                std::snprintf(message, sizeof(message), "Battery back above critical (%.2f V), still low", v);
            }
            break;
        case BatteryLevel::Critical:
            severity = LogLevel::Error;
            std::snprintf(message, sizeof(message), "Battery critical (%.2f V)%s", v, p.auto_sit ? ", sitting down" : "");
            break;
    }
    sit_pending = level_ == BatteryLevel::Critical && p.auto_sit;
    if (log) {
        log(severity, message);
    }
}

}  // namespace hexapod_behaviour
