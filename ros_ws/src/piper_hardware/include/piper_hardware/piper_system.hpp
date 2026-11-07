#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "piper_driver/arm.hpp"

namespace piper_hardware
{

/**
 * @class PiperSystem
 * @brief ros2_control system for the AgileX Piper over CAN: six joints and the gripper.
 * @details State interfaces per joint: position, velocity and effort. Command interface: position.
 * The gripper is `joint7`, one finger's travel in metres; the opening sent to the arm is twice it.
 *
 * Configuring opens the CAN interface and waits for feedback; nothing is sent. Activating with
 * `enable_on_activate` false is read-only. With it true, activating sets CAN control in MOVE J at
 * `speed_percent`, holds the measured pose and enables the motors; while active, every cycle sends
 * the commanded positions, clamped to the joint limits widened to include the pose at activation.
 * Deactivating holds the measured pose and leaves the motors enabled: this plugin never disables
 * them, so a raised arm never falls because of it.
 *
 * read() fails, stopping the controllers, when joint feedback is older than
 * `feedback_timeout_ms`, and, while commanding, when another sender commands the arm or the arm
 * reports a fault.
 */
class PiperSystem : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    static constexpr std::size_t kArmJoints = piper_driver::protocol::kJointCount;
    static constexpr std::size_t kGripper = kArmJoints;  // Index of joint7.

    struct Joint
    {
        hardware_interface::StateInterface::SharedPtr position;
        hardware_interface::StateInterface::SharedPtr velocity;
        hardware_interface::StateInterface::SharedPtr effort;
        hardware_interface::CommandInterface::SharedPtr command;
        double lower = 0.0;  // Command limits: rad, or m for the gripper.
        double upper = 0.0;
    };

    bool waitForFeedback(std::uint64_t frames, double timeout_s);
    void holdMeasuredPose();

    std::array<Joint, kArmJoints + 1> joints_{};
    std::unique_ptr<piper_driver::Arm> arm_;

    std::string can_interface_ = "can0";
    bool enable_on_activate_ = false;
    int speed_percent_ = 10;
    std::int64_t feedback_timeout_ns_ = 50'000'000;
    double gripper_effort_nm_ = 1.0;

    bool commanding_ = false;  // Motors enabled by this plugin and commands streamed.
    std::array<double, kArmJoints> target_rad_{};
    double gripper_target_m_ = -1.0;
    std::uint64_t seen_joint_frames_ = 0;
    std::int64_t fresh_at_ns_ = 0;
    std::uint64_t seen_foreign_ = 0;
    std::uint32_t cycle_ = 0;
    std::uint32_t send_failures_ = 0;
};

}  // namespace piper_hardware
