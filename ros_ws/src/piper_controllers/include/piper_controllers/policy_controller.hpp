#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

#include "piper_controllers/joint_ramp.hpp"
#include "piper_controllers/policy.hpp"

namespace piper_controllers
{

/**
 * @class PolicyController
 * @brief Moves the arm by a trained reach policy until the tool reaches a target pose.
 * @details The policy (ONNX, from the Isaac Lab task in `lab/`) runs at `policy_rate_hz` on the
 * observation [joint positions (rad), joint velocities (rad/s), target position (m), target
 * quaternion x y z w], all in the frame `frame`, and asks for joint positions (`action_offset`
 * plus `action_scale` times its actions). Every control cycle a JointRamp moves the position
 * commands towards them under `max_velocity` and `max_acceleration`, from the measured positions
 * on activation. The joints hold until the first target arrives on `~/target_pose` after
 * activation; a target in another frame is ignored, and the joints come to rest if inference
 * fails.
 *
 * Claims the `position` command and the `position` and `velocity` state interfaces of `joints`.
 *
 * @warning The policy was trained with the offsets, scale, caps and rate in its YAML: other values
 * change what it does.
 */
class PolicyController : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /// Target position (m) then quaternion x, y, z, w, in `frame_`.
    using Target = std::array<float, 7>;

    void readState();
    void onTarget(const geometry_msgs::msg::PoseStamped & pose);

    std::vector<std::string> joints_;
    std::string frame_;
    std::optional<Policy> policy_;
    std::optional<JointRamp> ramp_;
    // Control cycles per policy step, and where in that count this cycle is.
    unsigned int decimation_ = 1;
    unsigned int cycle_ = 0;
    std::vector<double> positions_rad_;
    std::vector<double> velocities_rad_s_;
    std::optional<Target> target_;
    // Written by the subscription, read without blocking by update().
    realtime_tools::RealtimeThreadSafeBox<std::optional<Target>> next_target_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
};

}  // namespace piper_controllers
