// Checks the engine never allocates once it's running, so it's safe on microcontrollers.
// Replaces the global operator new for this test binary only, counting every allocation.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>
#include <vector>

#include "gait_core/gait_engine.hpp"

namespace {
std::atomic<long> allocations{0};
}  // namespace

void* operator new(std::size_t size) {
    ++allocations;
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
    ++allocations;
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using gait_core::GaitEngine;
using gait_core::GaitOutput;
using gait_core::ImuSample;
using gait_core::JointFeedback;
using gait_core::KinematicSolver;
using gait_core::LogLevel;
using gait_core::NUM_LEGS;
using gait_core::Scalar;
using gait_core::Timestamp;
using gait_core::Vec3;

namespace {

int logged = 0;

/**
 * @brief Perfect servos, with the lowest feet carrying the weight so contact and load balancing run
*/
class Robot {
public:
    GaitEngine engine;
    KinematicSolver ik;
    Timestamp now = 1'000'000'000;
    JointFeedback feedback;
    bool imu = false;

    Robot() {
        engine.log = [](LogLevel, const char*) { ++logged; };
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            feedback.present[leg] = true;
            feedback.angles[leg] = {0, 0, 0};
        }
        feedback.hasEffort = true;
    }

    void run(double seconds) {
        const int ticks = static_cast<int>(seconds / 0.01 + 0.5);
        for (int i = 0; i < ticks; ++i) {
            setEfforts();
            feedback.stamp = now;
            engine.onJointStates(feedback);
            if (imu) {
                ImuSample sample;
                sample.stamp = now;
                sample.roll = static_cast<Scalar>(0.05);
                sample.pitch = static_cast<Scalar>(-0.04);
                engine.setImu(sample);
            }
            now += 10'000'000;
            const GaitOutput out = engine.update(static_cast<Scalar>(0.01), now);
            if (out.commanded) {
                for (int leg = 0; leg < NUM_LEGS; ++leg) {
                    feedback.angles[leg] = out.joints[leg];
                }
            }
        }
    }

private:
    void setEfforts() {
        Scalar footZ[NUM_LEGS];
        Scalar lowest = 1;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            footZ[leg] = ik.solveFK(feedback.angles[leg]).z();
            lowest = std::min(lowest, footZ[leg]);
        }
        int loaded = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            loaded += footZ[leg] <= lowest + static_cast<Scalar>(0.005) ? 1 : 0;
        }
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const Scalar fz = footZ[leg] <= lowest + static_cast<Scalar>(0.005) ? 25 / static_cast<Scalar>(loaded) : 0;
            feedback.effort[leg] = -ik.jacobian(feedback.angles[leg]).transpose() * Vec3(0, 0, fz);
        }
    }
};

}  // namespace

TEST(NoHeap, TheCounterSeesAllocations) {
    const long before = allocations.load();
    // A direct call: compilers may drop a new-expression whose result is never used
    void* p = ::operator new(16);
    ::operator delete(p);
    std::vector<int> v(10);
    EXPECT_EQ(allocations.load() - before, 2);
}

TEST(NoHeap, RunningTheGaitNeverAllocates) {
    Robot robot;
    robot.run(3.0);  // startup
    robot.engine.sit(false);
    robot.run(6.0);  // stand up

    const long before = allocations.load();
    logged = 0;

    robot.engine.setVelocity({static_cast<Scalar>(0.1), 0, 0});
    robot.run(3.0);
    robot.engine.setGait(gait_core::GAIT_RIPPLE);
    robot.run(3.0);
    robot.engine.setGait(gait_core::GAIT_WAVE);
    robot.engine.setMode(gait_core::MODE_STRAFE);
    robot.engine.setVelocity({static_cast<Scalar>(0.05), static_cast<Scalar>(0.05), static_cast<Scalar>(0.2)});
    robot.run(4.0);
    robot.engine.setGait(gait_core::GAIT_TRIPOD);
    robot.engine.setVelocity({0, 0, 0});
    robot.run(3.5);   // idle return, load balancing
    robot.imu = true;
    robot.engine.setMode(gait_core::MODE_NORMAL);
    robot.engine.setVelocity({static_cast<Scalar>(0.08), 0, 0});
    robot.run(2.0);   // walk leveling
    robot.engine.setVelocity({0, 0, 0});
    robot.engine.setHeight(static_cast<Scalar>(-0.11));
    robot.run(2.0);
    robot.engine.setMode(gait_core::MODE_TILT);
    robot.engine.setBodyPose(static_cast<Scalar>(0.1), static_cast<Scalar>(0.05), 0, Vec3(static_cast<Scalar>(0.02), 0, 0));
    robot.run(3.0);
    robot.engine.setMode(gait_core::MODE_NORMAL);
    robot.run(2.0);
    robot.engine.sit(true);
    robot.run(6.0);
    robot.engine.sit(false);
    robot.run(6.0);

    EXPECT_EQ(allocations.load() - before, 0) << "allocations while running the gait";
    EXPECT_GT(logged, 5) << "the log hook should have been called (gait changes, tilt, sitting)";
    EXPECT_GT(robot.engine.odometry().x(), 0.2) << "the robot should have walked";
}
