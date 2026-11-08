#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <vector>

#include "piper_lerobot/bridge_protocol.hpp"

namespace
{

using piper_lerobot::decodePositions;
using piper_lerobot::encodeImageSize;
using piper_lerobot::encodeState;
using piper_lerobot::jogVelocities;

std::vector<std::byte> bytesOf(std::span<const double> values)
{
    std::vector<std::byte> bytes(values.size_bytes());
    std::memcpy(bytes.data(), values.data(), values.size_bytes());
    return bytes;
}

TEST(BridgeProtocol, EncodesTheStampThenMeasuredThenCommanded)
{
    const std::array<double, 2> measured{0.1, -0.2};
    const std::array<double, 2> commanded{0.3, 0.035};
    const auto frame = encodeState(12.5, measured, commanded);
    ASSERT_EQ(frame.size(), 5 * sizeof(double));
    std::array<double, 5> values{};
    std::memcpy(values.data(), frame.data(), frame.size());
    EXPECT_EQ(values, (std::array<double, 5>{12.5, 0.1, -0.2, 0.3, 0.035}));
}

TEST(BridgeProtocol, EncodesTheImageSizeAsHeightThenWidth)
{
    const auto frame = encodeImageSize(480, 640);
    ASSERT_EQ(frame.size(), 2 * sizeof(std::uint32_t));
    std::array<std::uint32_t, 2> size{};
    std::memcpy(size.data(), frame.data(), frame.size());
    EXPECT_EQ(size, (std::array<std::uint32_t, 2>{480, 640}));
}

TEST(BridgeProtocol, DecodesPositions)
{
    const std::array<double, 3> sent{0.5, -1.0, 0.02};
    const auto positions = decodePositions(bytesOf(sent), 3);
    ASSERT_TRUE(positions.has_value());
    EXPECT_EQ(*positions, (std::vector<double>{0.5, -1.0, 0.02}));
}

TEST(BridgeProtocol, RefusesPositionsOfAnotherLengthOrNotFinite)
{
    const std::array<double, 2> two{0.5, -1.0};
    EXPECT_FALSE(decodePositions(bytesOf(two), 3).has_value());
    EXPECT_FALSE(decodePositions(std::span<const std::byte>(), 1).has_value());
    const std::array<double, 2> nan{0.5, std::numeric_limits<double>::quiet_NaN()};
    EXPECT_FALSE(decodePositions(bytesOf(nan), 2).has_value());
    const std::array<double, 2> inf{std::numeric_limits<double>::infinity(), 0.0};
    EXPECT_FALSE(decodePositions(bytesOf(inf), 2).has_value());
}

TEST(BridgeProtocol, JogsTowardsTheTargetUnderTheCap)
{
    const std::array<double, 3> target{1.0, 0.0, -0.1};
    const std::array<double, 3> measured{0.0, 0.0, 0.0};
    const auto velocities = jogVelocities(target, measured, 2.0, 0.5);
    ASSERT_EQ(velocities.size(), 3U);
    EXPECT_DOUBLE_EQ(velocities[0], 0.5);   // 2 rad/s wanted, capped
    EXPECT_DOUBLE_EQ(velocities[1], 0.0);   // there
    EXPECT_DOUBLE_EQ(velocities[2], -0.2);  // rad/s
}

TEST(BridgeProtocol, JogsOnlyTheJointsBothSidesHave)
{
    const std::array<double, 2> target{1.0, 1.0};
    const std::array<double, 1> measured{0.9};
    const auto velocities = jogVelocities(target, measured, 1.0, 1.0);
    ASSERT_EQ(velocities.size(), 1U);
    EXPECT_NEAR(velocities[0], 0.1, 1e-12);
}

}  // namespace
