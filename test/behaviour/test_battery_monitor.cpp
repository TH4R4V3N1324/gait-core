// Checks BatteryMonitor makes the same decisions as hexapod-ros2's behaviour_manager did

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "hexapod_behaviour/battery_monitor.hpp"

using gait_core::LogLevel;
using gait_core::Scalar;
using gait_core::Timestamp;
using hexapod_behaviour::BatteryLevel;
using hexapod_behaviour::BatteryMonitor;
using hexapod_behaviour::BatteryParams;
using hexapod_behaviour::BatteryStatus;

namespace {

constexpr Timestamp SECOND = 1'000'000'000;

/**
 * @brief A monitor fed at 10 Hz, as the daughter boards publish, updated at 2 Hz like the ROS node
*/
struct Battery {
    BatteryMonitor monitor;
    Timestamp now = SECOND;
    std::vector<std::pair<LogLevel, std::string>> logs;
    int alerts = 0;   // updates that said the level got worse

    explicit Battery(int monitors = 1, const BatteryParams& params = {}) : monitor(monitors, params) {
        monitor.log = [this](LogLevel level, const char* message) { logs.emplace_back(level, message); };
    }

    /**
     * @brief Feed every monitor the same voltage for a while, returning the last status
    */
    BatteryStatus hold(double volts, double seconds) {
        return hold(std::vector<double>{volts}, seconds);
    }

    /**
     * @brief Feed each monitor its own voltage (NaN for none) for a while, returning the last status
    */
    BatteryStatus hold(std::vector<double> volts, double seconds) {
        BatteryStatus status = monitor.update(now);
        const int ticks = static_cast<int>(seconds * 10 + 0.5);
        for (int i = 1; i <= ticks; ++i) {
            now += SECOND / 10;
            for (int m = 0; m < monitor.monitors(); ++m) {
                const double v = volts.size() == 1 ? volts[0] : volts.at(m);
                if (!std::isnan(v)) {
                    monitor.onReading(m, static_cast<Scalar>(v), Scalar(-2), now);
                }
            }
            if (i % 5 == 0) {
                status = monitor.update(now);
                alerts += status.worsened ? 1 : 0;
            }
        }
        return status;
    }
};

}  // namespace

TEST(BatteryMonitor, WithoutReadingsItStaysOkAndSaysSo) {
    Battery battery(2);
    const BatteryStatus status = battery.monitor.update(battery.now);
    EXPECT_EQ(status.level, BatteryLevel::Ok);
    EXPECT_FALSE(status.hasData);
    EXPECT_EQ(status.staleMonitors, 2);
    EXPECT_FALSE(status.sit);
}

TEST(BatteryMonitor, TheFirstReadingIsTakenAsIs) {
    Battery battery;
    battery.monitor.onReading(0, Scalar(11.1), Scalar(-1.5), battery.now);
    const BatteryStatus status = battery.monitor.update(battery.now);
    EXPECT_TRUE(status.hasData);
    EXPECT_NEAR(status.voltage, 11.1, 1e-5);
    EXPECT_NEAR(battery.monitor.reading(0).current, -1.5, 1e-6);
}

TEST(BatteryMonitor, AShortSagWhileWalkingIsFilteredOut) {
    Battery battery;
    battery.hold(11.0, 5);
    const BatteryStatus status = battery.hold(9.0, 0.5);   // well below critical, but only briefly
    EXPECT_EQ(status.level, BatteryLevel::Ok);
    EXPECT_GT(status.voltage, 10.7);
    EXPECT_EQ(battery.alerts, 0);
}

TEST(BatteryMonitor, LowVoltageWarnsOnceAndHysteresisHoldsIt) {
    Battery battery;
    battery.hold(11.0, 5);
    BatteryStatus status = battery.hold(10.3, 30);
    EXPECT_EQ(status.level, BatteryLevel::Warning);
    EXPECT_FALSE(status.sit);
    EXPECT_EQ(battery.alerts, 1);
    ASSERT_EQ(battery.logs.size(), 1u);
    EXPECT_EQ(battery.logs[0].first, LogLevel::Warn);
    EXPECT_EQ(battery.logs[0].second.rfind("Battery low (", 0), 0u) << battery.logs[0].second;

    // Above the threshold but inside the hysteresis band: still low
    status = battery.hold(10.6, 30);
    EXPECT_EQ(status.level, BatteryLevel::Warning);
    // Clear of the band: OK again, and recovering isn't an alert
    status = battery.hold(10.8, 30);
    EXPECT_EQ(status.level, BatteryLevel::Ok);
    EXPECT_EQ(battery.alerts, 1);
    EXPECT_EQ(battery.logs.back().first, LogLevel::Info);
}

TEST(BatteryMonitor, CriticalAsksToSitUntilTheRequestGoesOut) {
    Battery battery;
    battery.hold(11.0, 5);
    BatteryStatus status = battery.hold(9.5, 30);
    EXPECT_EQ(status.level, BatteryLevel::Critical);
    EXPECT_TRUE(status.sit);
    EXPECT_EQ(battery.logs.back().first, LogLevel::Error);
    EXPECT_NE(battery.logs.back().second.find("sitting down"), std::string::npos);

    // Keeps asking (the sit service might not be up yet), then stops once it's been sent
    EXPECT_TRUE(battery.monitor.update(battery.now).sit);
    battery.monitor.sitRequested();
    status = battery.hold(9.5, 10);
    EXPECT_EQ(status.level, BatteryLevel::Critical);
    EXPECT_FALSE(status.sit) << "it never asks twice, so the operator can stand it back up";
}

TEST(BatteryMonitor, RecoveringFromCriticalNeedsTheHysteresis) {
    Battery battery;
    battery.hold(9.5, 30);
    battery.monitor.sitRequested();
    EXPECT_EQ(battery.hold(10.0, 30).level, BatteryLevel::Critical);
    const BatteryStatus status = battery.hold(10.2, 30);
    EXPECT_EQ(status.level, BatteryLevel::Warning);
    EXPECT_FALSE(status.worsened);
    EXPECT_NE(battery.logs.back().second.find("back above critical"), std::string::npos);
}

TEST(BatteryMonitor, LeavingCriticalBeforeTheSitWentOutCancelsIt) {
    Battery battery;
    battery.hold(9.5, 30);
    EXPECT_FALSE(battery.hold(10.5, 30).sit);
    // Going critical again asks again
    EXPECT_TRUE(battery.hold(9.5, 30).sit);
}

TEST(BatteryMonitor, CriticalAlsoAsksToPowerOffUnlessTurnedOff) {
    Battery battery;
    BatteryStatus status = battery.hold(9.5, 30);
    EXPECT_TRUE(status.sit);
    EXPECT_TRUE(status.powerOff);
    EXPECT_NE(battery.logs.back().second.find("turning the servos off"), std::string::npos);

    BatteryParams params;
    params.power_off = false;
    Battery sitOnly(1, params);
    status = sitOnly.hold(9.5, 30);
    EXPECT_TRUE(status.sit);
    EXPECT_FALSE(status.powerOff);
    battery.monitor.sitRequested();
    EXPECT_FALSE(battery.monitor.update(battery.now).powerOff) << "only asked for with the sit";
}

TEST(BatteryMonitor, AutoSitOffOnlyWarns) {
    BatteryParams params;
    params.auto_sit = false;
    Battery battery(1, params);
    const BatteryStatus status = battery.hold(9.5, 30);
    EXPECT_EQ(status.level, BatteryLevel::Critical);
    EXPECT_FALSE(status.sit);
    EXPECT_EQ(battery.logs.back().second.find("sitting down"), std::string::npos);
}

TEST(BatteryMonitor, TheLowestMonitorSetsTheLevel) {
    Battery battery(2);
    const BatteryStatus status = battery.hold({11.2, 10.3}, 30);
    EXPECT_EQ(status.level, BatteryLevel::Warning);
    EXPECT_NEAR(status.voltage, 10.3, 0.01);
}

TEST(BatteryMonitor, AStaleMonitorIsIgnored) {
    Battery battery(2);
    battery.hold({11.2, 10.3}, 30);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // The low monitor stops reporting: after stale_timeout only the healthy one counts
    const BatteryStatus status = battery.hold({11.2, nan}, 10);
    EXPECT_EQ(status.staleMonitors, 1);
    EXPECT_TRUE(battery.monitor.stale(1, battery.now));
    EXPECT_NEAR(status.voltage, 11.2, 0.01);
    EXPECT_EQ(status.level, BatteryLevel::Ok);
}

TEST(BatteryMonitor, WithNoRecentDataTheLevelHolds) {
    Battery battery;
    battery.hold(10.3, 30);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const BatteryStatus status = battery.hold(nan, 10);
    EXPECT_FALSE(status.hasData);
    EXPECT_EQ(status.staleMonitors, 1);
    EXPECT_EQ(status.level, BatteryLevel::Warning) << "a dropped connection doesn't clear the warning";
}

TEST(BatteryMonitor, BadReadingsAreIgnored) {
    Battery battery(1);
    battery.monitor.onReading(0, std::numeric_limits<Scalar>::infinity(), 0, battery.now);
    battery.monitor.onReading(1, Scalar(9.0), 0, battery.now);   // no such monitor
    battery.monitor.onReading(-1, Scalar(9.0), 0, battery.now);
    EXPECT_FALSE(battery.monitor.update(battery.now).hasData);
}
