#include "piper_controllers/policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <iterator>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <urdf/model.hpp>

#include "piper_controllers/joint_ramp.hpp"
#include "piper_controllers/policy.hpp"

namespace piper_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

namespace
{

constexpr std::string_view kPosition{std::data(hardware_interface::HW_IF_POSITION)};
constexpr std::string_view kVelocity{std::data(hardware_interface::HW_IF_VELOCITY)};

}  // namespace

CallbackReturn PolicyController::on_init()
{
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("model_path", "");
    auto_declare<std::string>("frame", "base_link");
    auto_declare<double>("policy_rate_hz", 50.0);
    auto_declare<std::vector<double>>("action_offset", {});
    auto_declare<double>("action_scale", 0.5);
    auto_declare<double>("max_velocity", 1.0);
    auto_declare<double>("max_acceleration", 0.5);
    return CallbackReturn::SUCCESS;
}

InterfaceConfiguration PolicyController::command_interface_configuration() const
{
    InterfaceConfiguration config{.type = interface_configuration_type::INDIVIDUAL, .names = {}};
    for (const auto & joint : joints_)
    {
        config.names.push_back(std::format("{}/{}", joint, kPosition));
    }
    return config;
}

InterfaceConfiguration PolicyController::state_interface_configuration() const
{
    InterfaceConfiguration config{.type = interface_configuration_type::INDIVIDUAL, .names = {}};
    for (const auto & joint : joints_)
    {
        config.names.push_back(std::format("{}/{}", joint, kPosition));
        config.names.push_back(std::format("{}/{}", joint, kVelocity));
    }
    return config;
}

CallbackReturn PolicyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    const auto node = get_node();
    const auto logger = node->get_logger();
    joints_ = node->get_parameter("joints").as_string_array();
    frame_ = node->get_parameter("frame").as_string();
    const std::string model_path = node->get_parameter("model_path").as_string();
    const double policy_rate_hz = node->get_parameter("policy_rate_hz").as_double();
    if (joints_.empty())
    {
        RCLCPP_ERROR(logger, "The parameter joints names no joint");
        return CallbackReturn::ERROR;
    }

    urdf::Model model;
    if (!model.initString(get_robot_description()))
    {
        RCLCPP_ERROR(logger, "The robot description does not parse");
        return CallbackReturn::ERROR;
    }
    std::vector<double> lower_rad;
    std::vector<double> upper_rad;
    for (const auto & name : joints_)
    {
        const auto joint = model.getJoint(name);
        if (!joint || !joint->limits)
        {
            RCLCPP_ERROR(logger, "Joint %s has no limits in the robot description", name.c_str());
            return CallbackReturn::ERROR;
        }
        lower_rad.push_back(joint->limits->lower);
        upper_rad.push_back(joint->limits->upper);
    }

    // The policy steps a whole number of control cycles, as in training.
    const double cycles = static_cast<double>(get_update_rate()) / policy_rate_hz;
    decimation_ = static_cast<unsigned int>(std::lround(cycles));
    if (decimation_ < 1 || std::abs(cycles - decimation_) > 1e-6)
    {
        RCLCPP_ERROR(
            logger,
            "policy_rate_hz %.3f does not divide the update rate of %u Hz",
            policy_rate_hz,
            get_update_rate());
        return CallbackReturn::ERROR;
    }
    try
    {
        ramp_.emplace(
            std::move(lower_rad),
            std::move(upper_rad),
            node->get_parameter("action_offset").as_double_array(),
            node->get_parameter("action_scale").as_double(),
            node->get_parameter("max_velocity").as_double(),
            node->get_parameter("max_acceleration").as_double());
    }
    catch (const std::invalid_argument & error)
    {
        RCLCPP_ERROR(logger, "%s", error.what());
        return CallbackReturn::ERROR;
    }

    const std::size_t joints = joints_.size();
    auto policy = Policy::load(model_path, 2 * joints + std::tuple_size_v<Target>, joints);
    if (!policy)
    {
        RCLCPP_ERROR(logger, "%s", policy.error().c_str());
        return CallbackReturn::ERROR;
    }
    policy_.emplace(std::move(*policy));
    positions_rad_.assign(joints, 0.0);
    velocities_rad_s_.assign(joints, 0.0);

    target_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose",
        rclcpp::QoS(1).reliable(),
        [this](const geometry_msgs::msg::PoseStamped & pose) { onTarget(pose); });
    RCLCPP_INFO(
        logger,
        "Policy %s on %zu joints, every %u cycles",
        model_path.c_str(),
        joints,
        decimation_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn PolicyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!ramp_)
    {
        return CallbackReturn::ERROR;
    }
    JointRamp & ramp = *ramp_;
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        if (state_interfaces_[2 * i].get_prefix_name() != joints_[i]
            || state_interfaces_[2 * i + 1].get_interface_name() != kVelocity)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "State interfaces arrived out of order");
            return CallbackReturn::ERROR;
        }
    }
    readState();
    ramp.reset(positions_rad_);
    target_.reset();
    // A target sent before this activation is not acted on.
    next_target_.set(std::nullopt);
    cycle_ = 0;
    return CallbackReturn::SUCCESS;
}

CallbackReturn PolicyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    target_.reset();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type PolicyController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!policy_ || !ramp_)
    {
        return controller_interface::return_type::ERROR;
    }
    Policy & policy = *policy_;
    JointRamp & ramp = *ramp_;
    readState();
    if (const auto boxed = next_target_.try_get(); boxed && *boxed)
    {
        target_ = **boxed;
    }
    if (!target_)
    {
        ramp.stop();
    }
    else if (cycle_ == 0)
    {
        const Target & target = *target_;
        const std::span<float> observations = policy.observations();
        const std::size_t joints = joints_.size();
        for (std::size_t i = 0; i < joints; ++i)
        {
            observations[i] = static_cast<float>(positions_rad_[i]);
            observations[joints + i] = static_cast<float>(velocities_rad_s_[i]);
        }
        std::ranges::copy(
            target, std::next(observations.begin(), static_cast<std::ptrdiff_t>(2 * joints)));
        if (policy.run())
        {
            ramp.want(policy.actions());
        }
        else
        {
            ramp.stop();
        }
    }
    cycle_ = (cycle_ + 1) % decimation_;

    const std::span<const double> commands = ramp.step(period.seconds());
    for (std::size_t i = 0; i < commands.size(); ++i)
    {
        // A busy interface keeps the previous command for this cycle.
        std::ignore = command_interfaces_[i].set_value(commands[i]);
    }
    return controller_interface::return_type::OK;
}

void PolicyController::readState()
{
    // A reading the interface cannot give right now keeps the previous one.
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        if (const auto position = state_interfaces_[2 * i].get_optional())
        {
            positions_rad_[i] = *position;
        }
        if (const auto velocity = state_interfaces_[2 * i + 1].get_optional())
        {
            velocities_rad_s_[i] = *velocity;
        }
    }
}

void PolicyController::onTarget(const geometry_msgs::msg::PoseStamped & pose)
{
    const auto logger = get_node()->get_logger();
    if (pose.header.frame_id != frame_)
    {
        RCLCPP_WARN(
            logger,
            "Ignored a target in %s: targets are in %s",
            pose.header.frame_id.c_str(),
            frame_.c_str());
        return;
    }
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(p.x + p.y + p.z) || !(norm > 1e-6) || !std::isfinite(norm))
    {
        RCLCPP_WARN(logger, "Ignored a target with a non-finite position or no orientation");
        return;
    }
    next_target_.set(
        Target{
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z),
            static_cast<float>(q.x / norm),
            static_cast<float>(q.y / norm),
            static_cast<float>(q.z / norm),
            static_cast<float>(q.w / norm)});
}

}  // namespace piper_controllers

PLUGINLIB_EXPORT_CLASS(
    piper_controllers::PolicyController, controller_interface::ControllerInterface)
