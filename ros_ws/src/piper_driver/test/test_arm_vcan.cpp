// The driver against the fake firmware on a virtual CAN interface. Needs vcan0 up on the host
// (scripts/setup-host.sh); skipped without it.

#include <gtest/gtest.h>
#include <net/if.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <system_error>
#include <thread>
#include <utility>

#include "piper_driver/arm.hpp"
#include "piper_driver/can_bus.hpp"
#include "piper_driver/fake_arm.hpp"
#include "piper_driver/protocol.hpp"

using namespace std::chrono_literals;
using piper_driver::Arm;
using piper_driver::FakeArm;
namespace protocol = piper_driver::protocol;

namespace
{
constexpr const char * kInterface = "vcan0";

template <typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds timeout)
{
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end)
    {
        if (pred())
        {
            return true;
        }
        std::this_thread::sleep_for(5ms);
    }
    return pred();
}

class ArmOnVcan : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (if_nametoindex(kInterface) == 0)
        {
            GTEST_SKIP() << kInterface << " is not up";
        }
        FakeArm::Options options;
        options.interface = kInterface;
        options.start_rad = {0.1, 0.0, -0.2, 0.0, 0.3, 0.0};
        auto fake = FakeArm::start(options);
        ASSERT_TRUE(fake.has_value()) << fake.error().message();
        fake_ = std::move(*fake);
        auto arm = Arm::open(kInterface);
        ASSERT_TRUE(arm.has_value()) << arm.error().message();
        arm_ = std::move(*arm);
    }

    Arm & arm()
    {
        return *arm_;
    }
    FakeArm & fake()
    {
        return *fake_;
    }

private:
    std::unique_ptr<FakeArm> fake_;
    std::unique_ptr<Arm> arm_;
};
}  // namespace

TEST_F(ArmOnVcan, DecodesFeedback)
{
    ASSERT_TRUE(waitFor([&] { return arm().latest().joint_frames > 30; }, 1s));
    const auto & s = arm().latest();
    EXPECT_NEAR(s.position_rad[0], 0.1, 1e-4);
    EXPECT_NEAR(s.position_rad[4], 0.3, 1e-4);
    EXPECT_EQ(s.foreign_commands, 0U);
}

TEST_F(ArmOnVcan, HoldsWhileDisabledAndMovesWhenEnabled)
{
    const std::array<double, 6> target{0.2, 0.0, -0.2, 0.0, 0.3, 0.0};
    ASSERT_TRUE(arm().commandJoints(target));
    std::this_thread::sleep_for(100ms);
    EXPECT_NEAR(fake().positions()[0], 0.1, 1e-9) << "moved while disabled";

    ASSERT_TRUE(arm().enable(true));
    ASSERT_TRUE(arm().setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kJ, 50));
    ASSERT_TRUE(arm().commandJoints(target));
    ASSERT_TRUE(waitFor([&] { return std::abs(arm().latest().position_rad[0] - 0.2) < 1e-3; }, 2s));
    EXPECT_TRUE(fake().enabled());
}

TEST_F(ArmOnVcan, EmergencyStopHolds)
{
    ASSERT_TRUE(arm().enable(true));
    ASSERT_TRUE(arm().setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kJ, 10));
    ASSERT_TRUE(arm().emergencyStop(true));
    ASSERT_TRUE(arm().commandJoints(std::array<double, 6>{1.0, 0.0, -0.2, 0.0, 0.3, 0.0}));
    std::this_thread::sleep_for(100ms);
    EXPECT_NEAR(fake().positions()[0], 0.1, 1e-9);
    ASSERT_TRUE(waitFor([&] { return arm().latest().status.arm_status == 0x01; }, 1s));
}

TEST_F(ArmOnVcan, RejectsNonFiniteTargets)
{
    const auto sent = arm().commandJoints(std::array<double, 6>{NAN, 0, 0, 0, 0, 0});
    ASSERT_FALSE(sent.has_value());
    EXPECT_EQ(sent.error(), std::make_error_code(std::errc::invalid_argument));
}

TEST_F(ArmOnVcan, CountsForeignCommandFrames)
{
    auto other = piper_driver::CanBus::open(kInterface);
    ASSERT_TRUE(other.has_value());
    ASSERT_TRUE(other->send(protocol::encodeEnableAll(true)));
    EXPECT_TRUE(waitFor([&] { return arm().latest().foreign_commands == 1; }, 1s));
}

TEST_F(ArmOnVcan, ReadsFirmwareText)
{
    ASSERT_TRUE(arm().requestFirmware());
    EXPECT_TRUE(waitFor([&] { return arm().firmwareText() == "S-V1.8-2"; }, 1s));
}

TEST_F(ArmOnVcan, FollowsMitCommandsInMitMode)
{
    ASSERT_TRUE(arm().enable(true));
    ASSERT_TRUE(arm().setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kMit, 100, true));
    std::array<protocol::MitCommand, 6> commands{};
    for (std::size_t j = 0; j < commands.size(); ++j)
    {
        commands.at(j) = {.position_rad = fake().positions().at(j), .kp = 10.0, .kd = 0.8};
    }
    commands[1].position_rad = 0.3;
    ASSERT_TRUE(arm().commandMit(commands));
    EXPECT_TRUE(waitFor([&] { return std::abs(arm().latest().position_rad[1] - 0.3) < 1e-3; }, 2s));
    commands[2].kp = std::nan("");
    EXPECT_FALSE(arm().commandMit(commands).has_value());
}

TEST_F(ArmOnVcan, SetsAndReadsBackCollisionLevelsAndLoad)
{
    const std::array<std::uint8_t, 6> levels{1, 2, 3, 4, 5, 6};
    ASSERT_TRUE(arm().setCollisionLevels(levels));
    ASSERT_TRUE(arm().requestCollisionLevels());
    EXPECT_TRUE(waitFor([&] { return arm().latest().collision_levels_known; }, 1s));
    EXPECT_EQ(arm().latest().collision_levels, levels);
    ASSERT_TRUE(arm().setEndLoad(protocol::EndLoad::kFull));
    EXPECT_TRUE(waitFor([&] { return fake().endLoad() == 0x02; }, 1s));
}
