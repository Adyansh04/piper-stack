#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "piper_controllers/joint_ramp.hpp"

namespace
{

using piper_controllers::JointRamp;

constexpr double kDtS = 0.005;   // the 200 Hz control loop
constexpr double kScale = 0.5;   // rad per unit of action
constexpr double kMaxVel = 1.0;  // rad/s
constexpr double kMaxAcc = 0.5;  // rad/s^2

// Two joints with limits +-1 and +-2 rad, wanting 0.2 and -0.3 rad at a zero action.
JointRamp twoJoints()
{
    return JointRamp({-1.0, -2.0}, {1.0, 2.0}, {0.2, -0.3}, kScale, kMaxVel, kMaxAcc);
}

TEST(JointRamp, HoldsTheMeasuredPositionsUntilAnActionArrives)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> measured{0.3, -0.4};
    ramp.reset(measured);
    for (int i = 0; i < 100; ++i)
    {
        const auto targets = ramp.step(kDtS);
        EXPECT_DOUBLE_EQ(targets[0], 0.3);
        EXPECT_DOUBLE_EQ(targets[1], -0.4);
    }
}

// The law the reach task trains with (lab/piper_lab/actions.py), step by step.
TEST(JointRamp, MovesAsTheTrainingAction)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{0.0, 0.5};
    ramp.reset(start);
    const std::array<float, 2> actions{0.8F, -3.0F};
    ramp.want(actions);
    const std::array<double, 2> lower{-1.0, -2.0};
    const std::array<double, 2> upper{1.0, 2.0};
    const std::array<double, 2> wanted{
        0.2 + kScale * static_cast<double>(actions[0]),
        std::max(-0.3 + kScale * static_cast<double>(actions[1]), -2.0)};
    std::array<double, 2> velocity{0.0, 0.0};
    std::array<double, 2> target = start;
    for (int i = 0; i < 1500; ++i)
    {
        const auto targets = ramp.step(kDtS);
        for (std::size_t j = 0; j < 2; ++j)
        {
            const double error = wanted[j] - target[j];
            const double speed = std::min(
                {kMaxVel, std::sqrt(2.0 * kMaxAcc * std::abs(error)), std::abs(error) / kDtS});
            velocity[j] += std::clamp(
                std::copysign(speed, error) - velocity[j], -kMaxAcc * kDtS, kMaxAcc * kDtS);
            target[j] = std::clamp(target[j] + velocity[j] * kDtS, lower[j], upper[j]);
            EXPECT_NEAR(targets[j], target[j], 1e-12);  // rad
        }
    }
}

TEST(JointRamp, ArrivesAtTheWantedPositionAndStops)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{-0.5, 0.0};
    ramp.reset(start);
    const std::array<float, 2> actions{1.0F, 0.0F};  // 0.7 and -0.3 rad
    ramp.want(actions);
    double previous = start[0];
    double previous_velocity = 0.0;
    double highest = start[0];
    for (int i = 0; i < 2000; ++i)  // 10 s
    {
        const double target = ramp.step(kDtS)[0];
        const double velocity = (target - previous) / kDtS;
        EXPECT_LE(std::abs(velocity), kMaxVel + 1e-9);                             // rad/s
        EXPECT_LE(std::abs(velocity - previous_velocity), kMaxAcc * kDtS + 1e-9);  // rad/s
        highest = std::max(highest, target);
        previous = target;
        previous_velocity = velocity;
    }
    EXPECT_NEAR(previous, 0.7, 1e-6);  // rad
    // Deceleration lags by one capped step at the end: about 2 mrad of overshoot.
    EXPECT_LE(highest, 0.7 + 3e-3);
}

TEST(JointRamp, KeepsWantedPositionsAndTargetsInsideTheLimits)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{0.9, -1.9};
    ramp.reset(start);
    const std::array<float, 2> actions{100.0F, -100.0F};
    ramp.want(actions);
    for (int i = 0; i < 2000; ++i)
    {
        const auto targets = ramp.step(kDtS);
        EXPECT_LE(targets[0], 1.0);
        EXPECT_GE(targets[1], -2.0);
    }
    const auto targets = ramp.step(kDtS);
    EXPECT_NEAR(targets[0], 1.0, 1e-6);
    EXPECT_NEAR(targets[1], -2.0, 1e-6);
}

TEST(JointRamp, ClampsAResetOutsideTheLimits)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> measured{-1.000001, 2.5};
    ramp.reset(measured);
    const auto targets = ramp.step(kDtS);
    EXPECT_DOUBLE_EQ(targets[0], -1.0);
    EXPECT_DOUBLE_EQ(targets[1], 2.0);
}

TEST(JointRamp, StopsOnANonFiniteAction)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{0.0, 0.0};
    ramp.reset(start);
    const std::array<float, 2> actions{std::numeric_limits<float>::quiet_NaN(), 1.0F};
    ramp.want(actions);
    const auto targets = ramp.step(kDtS);
    EXPECT_DOUBLE_EQ(targets[0], 0.0);
    EXPECT_DOUBLE_EQ(targets[1], 0.0);
}

TEST(JointRamp, StopComesToRestAtTheCap)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{-1.0, 0.0};
    ramp.reset(start);
    const std::array<float, 2> actions{1.6F, -0.6F};  // 1.0 rad: far enough to keep accelerating
    ramp.want(actions);
    for (int i = 0; i < 200; ++i)  // 1 s at 0.5 rad/s^2: 0.5 rad/s
    {
        static_cast<void>(ramp.step(kDtS));
    }
    ramp.stop();
    double previous = ramp.step(kDtS)[0];
    int moving_steps = 1;
    for (; moving_steps < 1000; ++moving_steps)
    {
        const double target = ramp.step(kDtS)[0];
        if (target == previous)
        {
            break;
        }
        previous = target;
    }
    EXPECT_NEAR(moving_steps, 200, 2);  // 0.5 rad/s to rest at 0.5 rad/s^2: 1 s
}

TEST(JointRamp, MovesNothingOnAStepThatIsNotPositive)
{
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{0.1, 0.1};
    ramp.reset(start);
    const std::array<float, 2> actions{1.0F, 1.0F};
    ramp.want(actions);
    for (const double dt_s : {0.0, -0.005, std::numeric_limits<double>::quiet_NaN()})
    {
        const auto targets = ramp.step(dt_s);
        EXPECT_DOUBLE_EQ(targets[0], 0.1);
        EXPECT_DOUBLE_EQ(targets[1], 0.1);
    }
}

TEST(JointRamp, ReportsTheVelocitiesItMovesTheCommandsAt)
{
    // The policy runner sends these to MoveIt Servo, which integrates them as the ramp does.
    JointRamp ramp = twoJoints();
    const std::array<double, 2> start{0.0, 0.0};
    ramp.reset(start);
    const std::array<float, 2> actions{1.0F, -1.0F};
    ramp.want(actions);
    std::array<double, 2> previous{0.0, 0.0};
    for (int i = 0; i < 400; ++i)
    {
        const auto targets = ramp.step(kDtS);
        const auto velocities = ramp.velocities();
        ASSERT_EQ(velocities.size(), 2U);
        for (std::size_t j = 0; j < 2; ++j)
        {
            EXPECT_NEAR(targets[j] - previous[j], velocities[j] * kDtS, 1e-12);
            EXPECT_LE(std::abs(velocities[j]), kMaxVel + 1e-12);
            previous[j] = targets[j];
        }
    }
}

TEST(JointRamp, RefusesInconsistentLimitsOffsetsAndCaps)
{
    EXPECT_THROW(
        JointRamp({0.0}, {1.0, 2.0}, {0.0}, kScale, kMaxVel, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(
        JointRamp({0.0}, {1.0}, {0.0, 0.0}, kScale, kMaxVel, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(JointRamp({1.0}, {0.0}, {0.5}, kScale, kMaxVel, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(
        JointRamp({std::nan("")}, {1.0}, {0.5}, kScale, kMaxVel, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(JointRamp({0.0}, {1.0}, {0.5}, 0.0, kMaxVel, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(JointRamp({0.0}, {1.0}, {0.5}, kScale, 0.0, kMaxAcc), std::invalid_argument);
    EXPECT_THROW(JointRamp({0.0}, {1.0}, {0.5}, kScale, kMaxVel, -1.0), std::invalid_argument);
}

}  // namespace
