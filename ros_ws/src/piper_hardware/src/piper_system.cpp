#include "piper_hardware/piper_system.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "piper_driver/arm.hpp"
#include "piper_driver/protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace piper_hardware
{

namespace
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace protocol = piper_driver::protocol;

// The firmware may drop out of CAN control without a fresh mode frame now and then; 10 Hz at 200 Hz.
constexpr std::uint32_t kModeEveryCycles = 20;
// The gripper target is re-sent at least this often even when unchanged; 2 Hz at 200 Hz.
constexpr std::uint32_t kGripperEveryCycles = 100;
// Consecutive failed sends (a full transmit queue) before write() gives up.
constexpr std::uint32_t kMaxSendFailures = 20;

std::int64_t steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// hardware_interface declares the interface types as char arrays; views of them compare and
// format without decaying.
constexpr std::string_view kPosition{std::data(hardware_interface::HW_IF_POSITION)};
constexpr std::string_view kVelocity{std::data(hardware_interface::HW_IF_VELOCITY)};
constexpr std::string_view kEffort{std::data(hardware_interface::HW_IF_EFFORT)};

std::string interfaceName(const std::string & joint, std::string_view type)
{
    return std::format("{}/{}", joint, type);
}

std::string motionModeName(std::uint8_t mode)
{
    switch (mode)
    {
        case 1:
            return "MOVE CPV";
        case 2:
            return "MIT";
        default:
            return "MOVE J";
    }
}

protocol::EndLoad parseEndLoad(const std::string & load)
{
    if (load == "full")
    {
        return protocol::EndLoad::kFull;
    }
    if (load == "half")
    {
        return protocol::EndLoad::kHalf;
    }
    return protocol::EndLoad::kEmpty;
}

bool isTrue(const std::string & value)
{
    return value == "true" || value == "True" || value == "1";
}

// Faults that stop commanding; a refused target (beyond a firmware limit) only refuses that target.
bool isFault(std::uint8_t arm_status)
{
    switch (static_cast<protocol::ArmStatus>(arm_status))
    {
        case protocol::ArmStatus::kNormal:
        case protocol::ArmStatus::kNoSolution:
        case protocol::ArmStatus::kSingularity:
        case protocol::ArmStatus::kTargetBeyondLimit:
            return false;
        default:
            return true;
    }
}
}  // namespace

CallbackReturn PiperSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    if (info_.joints.size() != joints_.size())
    {
        RCLCPP_FATAL(
            get_logger(),
            "expected %zu joints (joint1 to joint6, then joint7), got %zu",
            joints_.size(),
            info_.joints.size());
        return CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        const auto & joint = info_.joints[i];
        const auto & commands = joint.command_interfaces;
        // Position first; arm joints may add velocity and effort, used in MIT mode only.
        const bool extra_ok = std::ranges::all_of(
            commands.begin() + (commands.empty() ? 0 : 1), commands.end(), [&](const auto & c) {
                return i < kArmJoints && (c.name == kVelocity || c.name == kEffort);
            });
        if (commands.empty() || commands[0].name != kPosition || !extra_ok)
        {
            RCLCPP_FATAL(
                get_logger(),
                "%s: a position command interface first, then optional velocity and effort",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        try
        {
            joints_[i].lower = std::stod(joint.command_interfaces[0].min);
            joints_[i].upper = std::stod(joint.command_interfaces[0].max);
        }
        catch (const std::exception &)
        {
            RCLCPP_FATAL(
                get_logger(), "%s: position command needs numeric min and max", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    try
    {
        const auto & p = info_.hardware_parameters;
        can_interface_ = p.at("can_interface");
        enable_on_activate_ = isTrue(p.at("enable_on_activate"));
        speed_percent_ = std::stoi(p.at("speed_percent"));
        feedback_timeout_ns_ = std::stoll(p.at("feedback_timeout_ms")) * 1'000'000;
        gripper_effort_nm_ = std::stod(p.at("gripper_effort_nm"));
        // Optional: defaults keep a description without them on MOVE J.
        const auto optional = [&p](const char * key, const char * fallback) {
            const auto it = p.find(key);
            return it == p.end() ? std::string{fallback} : it->second;
        };
        const std::string mode = optional("motion_mode", "joint");
        if (mode != "joint" && mode != "cpv" && mode != "mit")
        {
            throw std::invalid_argument("motion_mode must be joint, cpv or mit, not " + mode);
        }
        motion_mode_ = MotionMode::kJoint;
        if (mode == "cpv")
        {
            motion_mode_ = MotionMode::kCpv;
        }
        else if (mode == "mit")
        {
            motion_mode_ = MotionMode::kMit;
        }
        mit_kp_ = std::stod(optional("mit_kp", "10.0"));
        mit_kd_ = std::stod(optional("mit_kd", "0.8"));
        mit_max_torque_nm_ = std::stod(optional("mit_max_torque_nm", "2.0"));
        mit_max_velocity_rad_s_ = std::stod(optional("mit_max_velocity_rad_s", "3.0"));
        collision_level_ = std::stoi(optional("collision_level", "-1"));
        end_load_ = optional("end_load", "");
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(get_logger(), "missing or malformed hardware parameter: %s", e.what());
        return CallbackReturn::ERROR;
    }
    if (speed_percent_ < 1 || speed_percent_ > 100 || feedback_timeout_ns_ <= 0
        || gripper_effort_nm_ < 0.0 || gripper_effort_nm_ > protocol::kMaxGripperEffortNm)
    {
        RCLCPP_FATAL(
            get_logger(),
            "speed_percent must be 1-100, feedback_timeout_ms positive, "
            "gripper_effort_nm 0-5");
        return CallbackReturn::ERROR;
    }
    if (mit_kp_ < 0.0 || mit_kp_ > protocol::kMitKpMax || mit_kd_ < 0.0
        || mit_kd_ > protocol::kMitKdRange || mit_max_torque_nm_ < 0.0
        || mit_max_torque_nm_ > protocol::kMitTorqueNm || mit_max_velocity_rad_s_ <= 0.0
        || mit_max_velocity_rad_s_ > protocol::kMitVelocityRadS || collision_level_ < -1
        || std::cmp_greater(collision_level_, protocol::kMaxCollisionLevel)
        || (!end_load_.empty() && end_load_ != "empty" && end_load_ != "half"
            && end_load_ != "full"))
    {
        RCLCPP_FATAL(
            get_logger(),
            "mit_kp must be 0-500, mit_kd 0-5, mit_max_torque_nm 0-8, mit_max_velocity_rad_s "
            "0-45, collision_level -1-8, end_load empty, half or full");
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn PiperSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto arm = piper_driver::Arm::open(can_interface_);
    if (!arm)
    {
        RCLCPP_ERROR(
            get_logger(),
            "cannot open %s: %s",
            can_interface_.c_str(),
            arm.error().message().c_str());
        return CallbackReturn::ERROR;
    }
    arm_ = std::move(*arm);
    if (!waitForFeedback(10, 1.0))
    {
        RCLCPP_ERROR(
            get_logger(),
            "no joint feedback on %s within 1 s: is the arm powered and the bus up at "
            "1 Mbit/s?",
            can_interface_.c_str());
        arm_.reset();
        return CallbackReturn::ERROR;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        const std::string & name = info_.joints[i].name;
        joints_[i].position = get_state_interface_handle(interfaceName(name, kPosition));
        joints_[i].velocity = get_state_interface_handle(interfaceName(name, kVelocity));
        joints_[i].effort = get_state_interface_handle(interfaceName(name, kEffort));
        joints_[i].command = get_command_interface_handle(interfaceName(name, kPosition));
        for (const auto & command : info_.joints[i].command_interfaces)
        {
            if (command.name == kVelocity)
            {
                joints_[i].velocity_command =
                    get_command_interface_handle(interfaceName(name, kVelocity));
            }
            else if (command.name == kEffort)
            {
                joints_[i].effort_command =
                    get_command_interface_handle(interfaceName(name, kEffort));
            }
        }
    }
    const auto & state = arm_->latest();
    seen_foreign_ = state.foreign_commands;
    RCLCPP_INFO(
        get_logger(),
        "%s: feedback at %llu joint frames, %llu foreign command frames so far",
        can_interface_.c_str(),
        static_cast<unsigned long long>(state.joint_frames),
        static_cast<unsigned long long>(state.foreign_commands));
    return read(rclcpp::Time{}, rclcpp::Duration{0, 0}) == return_type::OK ? CallbackReturn::SUCCESS
                                                                           : CallbackReturn::ERROR;
}

CallbackReturn PiperSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    const auto & state = arm_->latest();
    std::ranges::copy(state.position_rad, target_rad_.begin());
    gripper_target_m_ = state.gripper.stroke_m / 2.0;
    // Controllers start from the measured pose, and the limits stretch to include it: the arm rests
    // slightly outside its firmware limits, and enabling must not move it.
    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        joints_[i].lower = std::min(joints_[i].lower, target_rad_[i]);
        joints_[i].upper = std::max(joints_[i].upper, target_rad_[i]);
        std::ignore = set_command(joints_[i].command, target_rad_[i], true);
    }
    std::ignore = set_command(joints_[kGripper].command, gripper_target_m_, true);

    if (!enable_on_activate_)
    {
        RCLCPP_WARN(
            get_logger(), "read-only: enable_on_activate is false, nothing is sent to the arm");
        return CallbackReturn::SUCCESS;
    }
    if (isFault(state.status.arm_status))
    {
        RCLCPP_ERROR(
            get_logger(), "not enabling: the arm reports status 0x%02X", state.status.arm_status);
        return CallbackReturn::ERROR;
    }
    target_velocity_rad_s_.fill(0.0);
    target_effort_nm_.fill(0.0);
    // Settings, then the mode and a hold target, so the motors come on holding the measured pose.
    if (!applySettings() || !sendMode() || !sendTargets() || !arm_->enable(true))
    {
        RCLCPP_ERROR(
            get_logger(), "sending the enable sequence on %s failed", can_interface_.c_str());
        return CallbackReturn::ERROR;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto & s = arm_->latest();
        const bool all_on = std::ranges::all_of(s.driver, [](const auto & d) {
            return (d.status_bits & protocol::kDriverEnabledBit) != 0;
        });
        if (all_on)
        {
            commanding_ = true;
            cycle_ = 0;
            send_failures_ = 0;
            seen_foreign_ = s.foreign_commands;
            RCLCPP_INFO(
                get_logger(),
                "motors enabled, holding the measured pose (%s, speed %d%%)",
                motionModeName(static_cast<std::uint8_t>(motion_mode_)).c_str(),
                speed_percent_);
            return CallbackReturn::SUCCESS;
        }
        std::ignore = sendTargets();
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    // Some motors may be on: hold the pose rather than disable and drop the arm.
    holdMeasuredPose();
    RCLCPP_ERROR(get_logger(), "not all six motors reported enabled within 2 s");
    return CallbackReturn::ERROR;
}

CallbackReturn PiperSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (commanding_)
    {
        holdMeasuredPose();
        commanding_ = false;
        RCLCPP_WARN(
            get_logger(),
            "deactivated: the arm holds its pose with the motors still enabled; disable "
            "them only with the arm at rest and supported");
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn PiperSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Closing the interface sends nothing: the arm keeps its state.
    arm_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PiperSystem::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (commanding_ && arm_)
    {
        holdMeasuredPose();
        commanding_ = false;
        RCLCPP_ERROR(get_logger(), "error: stopped commanding; the arm holds its last pose");
    }
    return CallbackReturn::SUCCESS;
}

return_type PiperSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    const auto & s = arm_->latest();
    const std::int64_t now_ns = steadyNs();
    if (s.joint_frames != seen_joint_frames_)
    {
        seen_joint_frames_ = s.joint_frames;
        fresh_at_ns_ = now_ns;
    }
    else if (now_ns - fresh_at_ns_ > feedback_timeout_ns_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "no joint feedback for over %lld ms",
            static_cast<long long>(feedback_timeout_ns_ / 1'000'000));
        return return_type::ERROR;
    }
    if (s.foreign_commands != seen_foreign_)
    {
        seen_foreign_ = s.foreign_commands;
        if (commanding_)
        {
            RCLCPP_ERROR(
                get_logger(), "another sender is commanding the arm on %s", can_interface_.c_str());
            return return_type::ERROR;
        }
        RCLCPP_WARN(
            get_logger(), "another sender is commanding the arm on %s", can_interface_.c_str());
    }
    if (commanding_ && isFault(s.status.arm_status))
    {
        RCLCPP_ERROR(get_logger(), "the arm reports status 0x%02X", s.status.arm_status);
        return return_type::ERROR;
    }

    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        std::ignore = set_state(joints_[i].position, s.position_rad[i], false);
        std::ignore = set_state(joints_[i].velocity, s.motor[i].velocity_rad_s, false);
        std::ignore = set_state(joints_[i].effort, s.motor[i].effort_nm, false);
    }
    std::ignore = set_state(joints_[kGripper].position, s.gripper.stroke_m / 2.0, false);
    std::ignore = set_state(joints_[kGripper].velocity, 0.0, false);
    std::ignore = set_state(joints_[kGripper].effort, s.gripper.effort_nm, false);
    return return_type::OK;
}

return_type PiperSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!commanding_)
    {
        return return_type::OK;
    }
    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        double command = std::numeric_limits<double>::quiet_NaN();
        // No new command (no controller, or the value is being written): keep the last target.
        if (get_command(joints_[i].command, command, false) && std::isfinite(command))
        {
            target_rad_[i] = std::clamp(command, joints_[i].lower, joints_[i].upper);
        }
        if (motion_mode_ == MotionMode::kMit)
        {
            double velocity = 0.0;
            double effort = 0.0;
            target_velocity_rad_s_[i] =
                joints_[i].velocity_command
                        && get_command(joints_[i].velocity_command, velocity, false)
                        && std::isfinite(velocity)
                    ? std::clamp(velocity, -mit_max_velocity_rad_s_, mit_max_velocity_rad_s_)
                    : 0.0;
            target_effort_nm_[i] = joints_[i].effort_command
                                           && get_command(joints_[i].effort_command, effort, false)
                                           && std::isfinite(effort)
                                       ? std::clamp(effort, -mit_max_torque_nm_, mit_max_torque_nm_)
                                       : 0.0;
        }
    }
    bool sent = sendTargets();
    if (++cycle_ % kModeEveryCycles == 0)
    {
        sent = sendMode() && sent;
    }
    double gripper = std::numeric_limits<double>::quiet_NaN();
    if (get_command(joints_[kGripper].command, gripper, false) && std::isfinite(gripper))
    {
        gripper = std::clamp(gripper, joints_[kGripper].lower, joints_[kGripper].upper);
        if (std::abs(gripper - gripper_target_m_) > 1e-4 || cycle_ % kGripperEveryCycles == 0)
        {
            gripper_target_m_ = gripper;
            sent = arm_->commandGripper(2.0 * gripper, gripper_effort_nm_).has_value() && sent;
        }
    }
    send_failures_ = sent ? 0 : send_failures_ + 1;
    if (send_failures_ > kMaxSendFailures)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%u consecutive sends failed on %s",
            send_failures_,
            can_interface_.c_str());
        return return_type::ERROR;
    }
    return return_type::OK;
}

bool PiperSystem::waitForFeedback(std::uint64_t frames, double timeout_s)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (arm_->latest().joint_frames >= frames)
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return false;
}

void PiperSystem::holdMeasuredPose()
{
    const auto & s = arm_->latest();
    std::ranges::copy(s.position_rad, target_rad_.begin());
    target_velocity_rad_s_.fill(0.0);
    target_effort_nm_.fill(0.0);
    std::ignore = sendTargets();
}

bool PiperSystem::sendMode()
{
    switch (motion_mode_)
    {
        case MotionMode::kJoint:
            return arm_->setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kJ, speed_percent_)
                .has_value();
        case MotionMode::kCpv:
            return arm_->setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kCpv, speed_percent_)
                .has_value();
        case MotionMode::kMit:
            // As piper_sdk's MIT demo: MOVE M with the MIT flag and no speed scaling.
            return arm_->setMode(protocol::CtrlMode::kCan, protocol::MoveMode::kMit, 0, true)
                .has_value();
    }
    return false;
}

bool PiperSystem::sendTargets()
{
    if (motion_mode_ != MotionMode::kMit)
    {
        return arm_->commandJoints(target_rad_).has_value();
    }
    std::array<protocol::MitCommand, kArmJoints> commands{};
    for (std::size_t i = 0; i < kArmJoints; ++i)
    {
        commands.at(i) = {
            .position_rad = target_rad_.at(i),
            .velocity_rad_s = target_velocity_rad_s_.at(i),
            .kp = mit_kp_,
            .kd = mit_kd_,
            .torque_nm = target_effort_nm_.at(i)};
    }
    return arm_->commandMit(commands).has_value();
}

bool PiperSystem::applySettings()
{
    bool sent = true;
    if (collision_level_ >= 0)
    {
        std::array<std::uint8_t, kArmJoints> levels{};
        levels.fill(static_cast<std::uint8_t>(collision_level_));
        sent = arm_->setCollisionLevels(levels).has_value();
    }
    if (!end_load_.empty())
    {
        sent = arm_->setEndLoad(parseEndLoad(end_load_)).has_value() && sent;
    }
    return sent;
}

}  // namespace piper_hardware

PLUGINLIB_EXPORT_CLASS(piper_hardware::PiperSystem, hardware_interface::SystemInterface)
