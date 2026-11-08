// Decoding is checked against frames captured from the real arm (firmware S-V1.8-2, 7 Nov 2026),
// encoding against the bytes AgileX's piper_sdk sends.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <numbers>

#include "piper_driver/protocol.hpp"

namespace protocol = piper_driver::protocol;

namespace
{
constexpr double kRadPerDeg = std::numbers::pi / 180.0;

protocol::Frame frame(std::uint32_t id, std::array<std::uint8_t, 8> data)
{
    return protocol::Frame{.id = id, .length = 8, .data = data};
}
}  // namespace

TEST(Protocol, DecodesCapturedJointFrames)
{
    std::array<double, 6> q{};
    ASSERT_TRUE(
        protocol::decodeJoints(frame(0x2A5, {0x00, 0x00, 0x0F, 0x5A, 0xFF, 0xFF, 0xF8, 0x35}), q));
    ASSERT_TRUE(
        protocol::decodeJoints(frame(0x2A6, {0x00, 0x00, 0x05, 0x78, 0xFF, 0xFF, 0xF7, 0xDC}), q));
    ASSERT_TRUE(
        protocol::decodeJoints(frame(0x2A7, {0x00, 0x00, 0x4D, 0x92, 0xFF, 0xFF, 0x92, 0x85}), q));
    const std::array<double, 6> expected_deg{3.930, -1.995, 1.400, -2.084, 19.858, -28.027};
    for (std::size_t j = 0; j < 6; ++j)
    {
        EXPECT_NEAR(q[j], expected_deg[j] * kRadPerDeg, 1e-9) << "joint " << j + 1;
    }
}

TEST(Protocol, IgnoresOtherFrames)
{
    std::array<double, 6> q{};
    EXPECT_FALSE(protocol::decodeJoints(frame(0x2A8, {}), q));
    EXPECT_FALSE(protocol::decodeStatus(frame(0x2A5, {})).has_value());
    EXPECT_FALSE(protocol::decodeGripper(frame(0x2A1, {})).has_value());
}

TEST(Protocol, DecodesCapturedGripperAndDriver)
{
    const auto gripper =
        protocol::decodeGripper(frame(0x2A8, {0x00, 0x00, 0x03, 0x48, 0, 0, 0, 0}));
    ASSERT_TRUE(gripper.has_value());
    // The check does not see that ASSERT_TRUE returns from the test when the value is missing.
    EXPECT_NEAR(gripper->stroke_m, 0.000840, 1e-12);  // NOLINT(bugprone-unchecked-optional-access)

    std::array<protocol::DriverFeedback, 6> drivers{};
    ASSERT_TRUE(
        protocol::decodeDriver(frame(0x261, {0x00, 0xE6, 0x00, 0x1C, 0x16, 0x00, 0, 0}), drivers));
    EXPECT_NEAR(drivers[0].voltage_v, 23.0, 1e-9);
    EXPECT_EQ(drivers[0].driver_temp_c, 28);
    EXPECT_EQ(drivers[0].motor_temp_c, 22);
}

TEST(Protocol, DecodesMotorEffortWithAgilexCoefficients)
{
    std::array<protocol::MotorFeedback, 6> motors{};
    // 1000 mA on joint 1 and on joint 4, -0.5 rad/s on joint 4.
    ASSERT_TRUE(protocol::decodeMotor(frame(0x251, {0x00, 0x00, 0x03, 0xE8, 0, 0, 0, 0}), motors));
    ASSERT_TRUE(protocol::decodeMotor(frame(0x254, {0xFE, 0x0C, 0x03, 0xE8, 0, 0, 0, 0}), motors));
    EXPECT_NEAR(motors[0].effort_nm, 1.18125, 1e-9);
    EXPECT_NEAR(motors[3].effort_nm, 0.95844, 1e-9);
    EXPECT_NEAR(motors[3].velocity_rad_s, -0.5, 1e-9);
}

TEST(Protocol, DecodesStatusErrorBits)
{
    const auto status =
        protocol::decodeStatus(frame(0x2A1, {0x01, 0x04, 0x01, 0, 0x01, 0, 0x06, 0x01}));
    ASSERT_TRUE(status.has_value());
    // NOLINTBEGIN(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when it is empty
    EXPECT_EQ(status->ctrl_mode, 0x01);
    EXPECT_EQ(
        status->arm_status, static_cast<std::uint8_t>(protocol::ArmStatus::kTargetBeyondLimit));
    EXPECT_EQ(status->joint_beyond_limit, 0x06);  // joints 2 and 3
    EXPECT_EQ(status->joint_comm_error, 0x01);    // joint 1
    // NOLINTEND(bugprone-unchecked-optional-access)
}

TEST(Protocol, EncodesCommandsLikeTheSdk)
{
    EXPECT_EQ(protocol::encodeEnableAll(true).data, (std::array<std::uint8_t, 8>{0x07, 0x02}));
    EXPECT_EQ(protocol::encodeEnableAll(false).data, (std::array<std::uint8_t, 8>{0x07, 0x01}));
    EXPECT_EQ(protocol::encodeEnableAll(true).id, 0x471U);
    const auto mode = protocol::encodeMode(protocol::CtrlMode::kCan, protocol::MoveMode::kJ, 250);
    EXPECT_EQ(mode.id, 0x151U);
    EXPECT_EQ(mode.data, (std::array<std::uint8_t, 8>{0x01, 0x01, 100, 0x00}));
    EXPECT_EQ(protocol::encodeEmergencyStop(true).data[0], 0x01);
    EXPECT_EQ(protocol::encodeEmergencyStop(false).data[0], 0x02);
    EXPECT_EQ(protocol::encodeFirmwareQuery().id, 0x4AFU);
}

TEST(Protocol, JointTargetsRoundTrip)
{
    const std::array<double, 6> target{0.1, -0.2, -1.5, 0.75, -1.2, 3.0};
    const auto frames = protocol::encodeJoints(target);
    std::array<double, 6> decoded{};
    for (const auto & f : frames)
    {
        ASSERT_TRUE(protocol::decodeJointTargets(f, decoded));
    }
    for (std::size_t j = 0; j < 6; ++j)
    {
        // The wire resolution is 0.001 deg.
        EXPECT_NEAR(decoded[j], target[j], 0.0005 * kRadPerDeg) << "joint " << j + 1;
    }
    EXPECT_EQ(frames[0].id, 0x155U);
    EXPECT_EQ(frames[2].id, 0x157U);
}

TEST(Protocol, GripperEffortIsClamped)
{
    const auto f = protocol::encodeGripper(0.05, 9.0);
    EXPECT_EQ(protocol::detail::beI32(f.data, 0), 50000);  // 0.001 mm
    EXPECT_EQ(protocol::detail::beU16(f.data, 4), 5000);   // 0.001 N m, at the 5 N m cap
    EXPECT_EQ(f.data[6], 0x01);
}

TEST(Protocol, ClassifiesCommandIds)
{
    for (const std::uint32_t id : {0x150U, 0x151U, 0x155U, 0x159U, 0x15AU, 0x471U, 0x474U})
    {
        EXPECT_TRUE(protocol::isCommand(id)) << std::hex << id;
    }
    for (const std::uint32_t id : {0x2A1U, 0x2A5U, 0x251U, 0x261U, 0x472U, 0x473U, 0x4AFU})
    {
        EXPECT_FALSE(protocol::isCommand(id)) << std::hex << id;
    }
}

// Expected bytes worked out by hand from piper_sdk's MIT encoder (FloatToUint truncation, nibble
// packing, XOR checksum in the low nibble of byte 7).
TEST(Protocol, EncodesMitLikeTheSdk)
{
    const auto a = protocol::encodeMit(
        0, {.position_rad = 0.0, .velocity_rad_s = 0.0, .kp = 10.0, .kd = 0.8, .torque_nm = 0.0});
    EXPECT_EQ(a.id, 0x15AU);
    EXPECT_EQ(
        a.data, (std::array<std::uint8_t, 8>{0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x77, 0xFD}));
    const auto b = protocol::encodeMit(
        2, {.position_rad = -1.2, .velocity_rad_s = 0.5, .kp = 50.0, .kd = 1.5, .torque_nm = -2.0});
    EXPECT_EQ(b.id, 0x15CU);
    EXPECT_EQ(
        b.data, (std::array<std::uint8_t, 8>{0x73, 0xB5, 0x81, 0x61, 0x99, 0xA6, 0x55, 0xFC}));
}

TEST(Protocol, DecodesMitWithinOneQuantisationStep)
{
    const protocol::MitCommand in{
        .position_rad = 1.234, .velocity_rad_s = -3.5, .kp = 42.0, .kd = 0.6, .torque_nm = 1.25};
    const auto out = protocol::decodeMit(protocol::encodeMit(5, in));
    ASSERT_TRUE(out.has_value());
    // NOLINTBEGIN(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when it is empty
    EXPECT_EQ(out->first, 5U);
    EXPECT_NEAR(out->second.position_rad, in.position_rad, 25.0 / 65535.0);
    EXPECT_NEAR(out->second.velocity_rad_s, in.velocity_rad_s, 90.0 / 4095.0);
    EXPECT_NEAR(out->second.kp, in.kp, 500.0 / 4095.0);
    EXPECT_NEAR(out->second.kd, in.kd, 10.0 / 4095.0);
    EXPECT_NEAR(out->second.torque_nm, in.torque_nm, 16.0 / 255.0);
    // NOLINTEND(bugprone-unchecked-optional-access)
}

TEST(Protocol, ClampsMitFieldsAndRejectsBadChecksums)
{
    auto frame = protocol::encodeMit(
        1,
        {.position_rad = 99.0, .velocity_rad_s = -99.0, .kp = 9e3, .kd = -9.0, .torque_nm = 50.0});
    const auto out = protocol::decodeMit(frame);
    ASSERT_TRUE(out.has_value());
    // NOLINTBEGIN(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when it is empty
    EXPECT_NEAR(out->second.position_rad, protocol::kMitPositionRad, 1e-9);
    EXPECT_NEAR(out->second.velocity_rad_s, -protocol::kMitVelocityRadS, 1e-9);
    EXPECT_NEAR(out->second.kp, protocol::kMitKpMax, 1e-9);
    EXPECT_NEAR(out->second.kd, -protocol::kMitKdRange, 1e-9);
    EXPECT_NEAR(out->second.torque_nm, protocol::kMitTorqueNm, 1e-9);
    // NOLINTEND(bugprone-unchecked-optional-access)
    frame.data[7] ^= 0x01;
    EXPECT_FALSE(protocol::decodeMit(frame).has_value());
}

TEST(Protocol, EncodesModeCollisionLevelsAndEndLoad)
{
    EXPECT_EQ(
        protocol::encodeMode(protocol::CtrlMode::kCan, protocol::MoveMode::kMit, 100, true).data[3],
        0xAD);
    EXPECT_EQ(
        protocol::encodeMode(protocol::CtrlMode::kCan, protocol::MoveMode::kJ, 10).data[3], 0x00);
    const std::array<std::uint8_t, 6> levels{0, 1, 4, 8, 9, 200};
    const auto frame = protocol::encodeCollisionLevels(levels);
    EXPECT_EQ(frame.id, 0x47AU);
    const auto back = protocol::decodeCollisionLevels(frame);
    ASSERT_TRUE(back.has_value());
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when empty
    EXPECT_EQ(*back, (std::array<std::uint8_t, 6>{0, 1, 4, 8, 8, 8}));
    EXPECT_EQ(protocol::encodeCollisionLevelQuery().data[0], 0x02);
    const auto load = protocol::encodeEndLoad(protocol::EndLoad::kHalf);
    EXPECT_EQ(load.id, 0x477U);
    EXPECT_EQ(load.data[3], 0xAE);
    EXPECT_EQ(load.data[4], 0x01);
    EXPECT_TRUE(protocol::isCommand(0x15F));
    EXPECT_TRUE(protocol::isCommand(0x47A));
    EXPECT_FALSE(protocol::isCommand(0x47B));
}
