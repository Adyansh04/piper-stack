#include "piper_driver/fake_arm.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <stop_token>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>

#include "piper_driver/can_bus.hpp"
#include "piper_driver/protocol.hpp"

namespace piper_driver
{

namespace
{
using protocol::Frame;
namespace id = protocol::id;

Frame pairFrame(std::uint32_t can_id, double a, double b, double scale)
{
    Frame f{.id = can_id, .length = 8, .data = {}};
    protocol::detail::putI32(f.data, 0, protocol::detail::toI32(a * scale));
    protocol::detail::putI32(f.data, 4, protocol::detail::toI32(b * scale));
    return f;
}
}  // namespace

std::expected<std::unique_ptr<FakeArm>, std::error_code> FakeArm::start(const Options & options)
{
    auto bus = CanBus::open(options.interface);
    if (!bus)
    {
        return std::unexpected(bus.error());
    }
    return std::unique_ptr<FakeArm>(new FakeArm(std::move(*bus), options));
}

FakeArm::FakeArm(CanBus bus, const Options & options)
    : bus_(std::move(bus))
    , options_(options)
    , position_rad_(options.start_rad)
    , target_rad_(options.start_rad)
    , runner_([this](const std::stop_token & stop) { run(stop); })
{}

FakeArm::~FakeArm() = default;

std::array<double, protocol::kJointCount> FakeArm::positions() const
{
    const std::scoped_lock lock(mutex_);
    return position_rad_;
}

bool FakeArm::enabled() const
{
    const std::scoped_lock lock(mutex_);
    return enabled_;
}

std::array<std::uint8_t, protocol::kJointCount> FakeArm::collisionLevels() const
{
    const std::scoped_lock lock(mutex_);
    return collision_levels_;
}

std::uint8_t FakeArm::endLoad() const
{
    const std::scoped_lock lock(mutex_);
    return end_load_;
}

void FakeArm::run(const std::stop_token & stop)
{
    const double dt_s = std::chrono::duration<double>(options_.period).count();
    auto next = std::chrono::steady_clock::now();
    for (std::uint64_t cycle = 0; !stop.stop_requested(); ++cycle)
    {
        while (auto received = bus_.receive(std::chrono::milliseconds{0}))
        {
            if (!received->error)
            {
                apply(received->frame);
            }
        }
        step(dt_s);
        sendFeedback(cycle);
        next += options_.period;
        std::this_thread::sleep_until(next);
    }
}

void FakeArm::apply(const Frame & f)
{
    const std::scoped_lock lock(mutex_);
    switch (f.id)
    {
        case id::kMotorEnable:
            if (f.data[0] == 0x07 || f.data[0] == 0xFF)
            {
                enabled_ = f.data[1] == 0x02;
                target_rad_ = position_rad_;
            }
            return;
        case id::kMotionCtrl2:
            ctrl_mode_ = f.data[0];
            move_mode_ = f.data[1];
            speed_percent_ = std::min<int>(f.data[2], 100);
            mit_ = f.data[3] == 0xAD;
            return;
        case id::kMotionCtrl1:
            if (f.data[0] == 0x01)
            {
                stopped_ = true;
                target_rad_ = position_rad_;
            }
            else if (f.data[0] == 0x02)
            {
                stopped_ = false;
            }
            return;
        case id::kGripperCtrl:
            gripper_stroke_m_ = protocol::detail::beI32(f.data, 0) * protocol::kMetrePerMicrometre;
            return;
        case id::kCollisionLevel:
            if (auto levels = protocol::decodeCollisionLevels(f))
            {
                collision_levels_ = *levels;
            }
            return;
        case id::kParamQuery:
            if (f.data[0] == 0x02)
            {
                Frame reply{.id = id::kCollisionReply, .length = 8, .data = {}};
                std::ranges::copy(collision_levels_, reply.data.begin());
                std::ignore = bus_.send(reply);
            }
            if (f.data[3] == 0xAE)
            {
                end_load_ = f.data[4];
            }
            return;
        case id::kFirmware:
            if (f.data[0] == 0x01)
            {
                for (const char * chunk : {"S-V1.8-2", "fake-arm"})
                {
                    Frame reply{.id = id::kFirmware, .length = 8, .data = {}};
                    std::copy_n(chunk, 8, reply.data.begin());
                    std::ignore = bus_.send(reply);
                }
            }
            return;
        default:
            if (stopped_)
            {
                return;
            }
            if (auto mit = protocol::decodeMit(f); mit && mit_)
            {
                // A stiff servo stands in for the motor's impedance: it tracks the position.
                target_rad_.at(mit->first) = mit->second.position_rad;
            }
            else if (!mit_)
            {
                protocol::decodeJointTargets(f, target_rad_);
            }
    }
}

void FakeArm::step(double dt_s)
{
    const std::scoped_lock lock(mutex_);
    const bool position_mode = move_mode_ == static_cast<std::uint8_t>(protocol::MoveMode::kJ)
                               || move_mode_ == static_cast<std::uint8_t>(protocol::MoveMode::kCpv);
    const bool moving = enabled_ && !stopped_ && ctrl_mode_ == 0x01 && (position_mode || mit_);
    if (!moving)
    {
        return;
    }
    // MIT commands bypass the firmware's speed scaling.
    const double max_step =
        options_.max_speed_rad_s * (mit_ ? 100.0 : speed_percent_) / 100.0 * dt_s;
    for (std::size_t j = 0; j < protocol::kJointCount; ++j)
    {
        position_rad_[j] += std::clamp(target_rad_[j] - position_rad_[j], -max_step, max_step);
    }
}

void FakeArm::sendFeedback(std::uint64_t cycle)
{
    std::array<double, protocol::kJointCount> q{};
    double stroke_m = 0.0;
    Frame status{.id = id::kStatus, .length = 8, .data = {}};
    {
        const std::scoped_lock lock(mutex_);
        q = position_rad_;
        stroke_m = gripper_stroke_m_;
        const bool arrived = std::ranges::equal(position_rad_, target_rad_);
        status.data = {
            ctrl_mode_,
            static_cast<std::uint8_t>(stopped_ ? 0x01 : 0x00),
            move_mode_,
            0,
            static_cast<std::uint8_t>(arrived ? 0x00 : 0x01),
            0,
            0,
            0};
    }
    std::ignore = bus_.send(status);
    for (std::uint32_t i = 0; i < 3; ++i)
    {
        std::ignore = bus_.send(pairFrame(
            id::kJoint12 + i,
            q[2 * std::size_t{i}],
            q[(2 * std::size_t{i}) + 1],
            protocol::kMilliDegPerRad));
    }
    Frame gripper{.id = id::kGripper, .length = 8, .data = {}};
    protocol::detail::putI32(
        gripper.data, 0, protocol::detail::toI32(stroke_m / protocol::kMetrePerMicrometre));
    std::ignore = bus_.send(gripper);
    // Driver frames at a fifth of the rate, like the arm's 40 Hz.
    if (cycle % 5 == 0)
    {
        const bool on = enabled();
        for (std::uint32_t j = 0; j < protocol::kJointCount; ++j)
        {
            const Frame driver{
                .id = id::kLowSpeed1 + j,
                .length = 8,
                .data = {
                    0x00,
                    0xE6,
                    0x00,
                    0x1C,
                    0x16,
                    static_cast<std::uint8_t>(on ? protocol::kDriverEnabledBit : 0x00),
                    0,
                    0}};
            std::ignore = bus_.send(driver);
        }
    }
}

}  // namespace piper_driver
