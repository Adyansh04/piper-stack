#include "piper_driver/arm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <iterator>
#include <memory>
#include <mutex>
#include <span>
#include <stop_token>
#include <string>
#include <system_error>
#include <utility>

#include "piper_driver/can_bus.hpp"
#include "piper_driver/protocol.hpp"

namespace piper_driver
{

std::expected<std::unique_ptr<Arm>, std::error_code> Arm::open(const std::string & interface)
{
    auto bus = CanBus::open(interface);
    if (!bus)
    {
        return std::unexpected(bus.error());
    }
    return std::unique_ptr<Arm>(new Arm(std::move(*bus)));
}

Arm::Arm(CanBus bus)
    : bus_(std::move(bus)), receiver_([this](const std::stop_token & stop) { receiveLoop(stop); })
{}

Arm::~Arm() = default;

void Arm::receiveLoop(const std::stop_token & stop)
{
    using protocol::id::kFirmware;
    ArmState state;
    // Short enough that a stop request ends the thread promptly.
    constexpr std::chrono::milliseconds kWait{50};
    while (!stop.stop_requested())
    {
        auto received = bus_.receive(kWait);
        if (!received)
        {
            continue;
        }
        if (received->error)
        {
            ++state.bus_errors;
            state_.write(state);
            continue;
        }
        const protocol::Frame & f = received->frame;
        ++state.frames;
        if (protocol::decodeJoints(f, state.position_rad))
        {
            ++state.joint_frames;
            state.joint_stamp_ns = received->stamp_ns;
        }
        else if (auto status = protocol::decodeStatus(f))
        {
            state.status = *status;
        }
        else if (auto gripper = protocol::decodeGripper(f))
        {
            state.gripper = *gripper;
        }
        else if (protocol::decodeMotor(f, state.motor) || protocol::decodeDriver(f, state.driver))
        {
            // Decoded into state in place.
        }
        else if (auto levels = protocol::decodeCollisionLevels(f);
                 levels && f.id == protocol::id::kCollisionReply)
        {
            state.collision_levels = *levels;
            state.collision_levels_known = true;
        }
        else if (f.id == kFirmware && f.data[0] != 0x01)
        {
            const std::scoped_lock lock(firmware_mutex_);
            // Bounded: replies are a few frames, and a stray query must not grow this forever.
            if (firmware_raw_.size() < 512)
            {
                firmware_raw_.append(f.data.begin(), std::next(f.data.begin(), f.length));
            }
        }
        else if (protocol::isCommand(f.id))
        {
            // Our own frames are not looped back to this socket: anything here came from someone else.
            ++state.foreign_commands;
        }
        state_.write(state);
    }
}

std::expected<void, std::error_code> Arm::send(const protocol::Frame & frame) noexcept
{
    return bus_.send(frame);
}

std::expected<void, std::error_code> Arm::enable(bool on) noexcept
{
    return send(protocol::encodeEnableAll(on));
}

std::expected<void, std::error_code> Arm::setMode(
    protocol::CtrlMode ctrl, protocol::MoveMode move, int speed_percent, bool mit) noexcept
{
    return send(protocol::encodeMode(ctrl, move, speed_percent, mit));
}

std::expected<void, std::error_code> Arm::commandMit(
    std::span<const protocol::MitCommand, protocol::kJointCount> commands) noexcept
{
    const auto finite = [](const protocol::MitCommand & c) {
        return std::isfinite(c.position_rad) && std::isfinite(c.velocity_rad_s)
               && std::isfinite(c.kp) && std::isfinite(c.kd) && std::isfinite(c.torque_nm);
    };
    if (!std::ranges::all_of(commands, finite))
    {
        return std::unexpected(std::make_error_code(std::errc::invalid_argument));
    }
    for (std::size_t joint = 0; joint < protocol::kJointCount; ++joint)
    {
        if (auto sent = send(protocol::encodeMit(joint, commands[joint])); !sent)
        {
            return sent;
        }
    }
    return {};
}

std::expected<void, std::error_code> Arm::setCollisionLevels(
    std::span<const std::uint8_t, protocol::kJointCount> levels) noexcept
{
    return send(protocol::encodeCollisionLevels(levels));
}

std::expected<void, std::error_code> Arm::requestCollisionLevels() noexcept
{
    return send(protocol::encodeCollisionLevelQuery());
}

std::expected<void, std::error_code> Arm::setEndLoad(protocol::EndLoad load) noexcept
{
    return send(protocol::encodeEndLoad(load));
}

std::expected<void, std::error_code> Arm::commandJoints(
    std::span<const double, protocol::kJointCount> position_rad) noexcept
{
    if (!std::ranges::all_of(position_rad, [](double v) { return std::isfinite(v); }))
    {
        return std::unexpected(std::make_error_code(std::errc::invalid_argument));
    }
    for (const auto & frame : protocol::encodeJoints(position_rad))
    {
        if (auto sent = send(frame); !sent)
        {
            return sent;
        }
    }
    return {};
}

std::expected<void, std::error_code> Arm::commandGripper(double stroke_m, double effort_nm) noexcept
{
    if (!std::isfinite(stroke_m) || !std::isfinite(effort_nm))
    {
        return std::unexpected(std::make_error_code(std::errc::invalid_argument));
    }
    return send(protocol::encodeGripper(stroke_m, effort_nm));
}

std::expected<void, std::error_code> Arm::emergencyStop(bool stop) noexcept
{
    return send(protocol::encodeEmergencyStop(stop));
}

std::expected<void, std::error_code> Arm::requestFirmware() noexcept
{
    {
        const std::scoped_lock lock(firmware_mutex_);
        firmware_raw_.clear();
    }
    return send(protocol::encodeFirmwareQuery());
}

std::string Arm::firmwareText() const
{
    const std::scoped_lock lock(firmware_mutex_);
    // AgileX's own reader takes the 8 characters from "S-V": the software version.
    const auto at = firmware_raw_.find("S-V");
    return at == std::string::npos ? std::string{} : firmware_raw_.substr(at, 8);
}

}  // namespace piper_driver
