#include "piper_skills/arm_motion.hpp"

#include <algorithm>
#include <cstddef>
#include <expected>
#include <format>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "piper_skills/srdf_states.hpp"

#include <moveit/robot_state/conversions.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace piper_skills
{

namespace
{

// The move_group servers must be up when a skill first moves; the skills node starts with the
// stack, so this only waits out move_group's own start.
constexpr double kServerWaitS = 30.0;

}  // namespace

ArmMotion::ArmMotion(const rclcpp::Node::SharedPtr & node, MotionSettings settings)
    : node_(node)
    , settings_(std::move(settings))
    , group_(
          node,
          settings_.group,
          std::shared_ptr<tf2_ros::Buffer>(),
          rclcpp::Duration::from_seconds(kServerWaitS))
{
    group_.setEndEffectorLink(settings_.tcp_link);
    group_.setPoseReferenceFrame(settings_.planning_frame);
    group_.setPlanningTime(settings_.planning_time_s);
    group_.setNumPlanningAttempts(settings_.attempts);
    ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    gripper_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(
        node_, settings_.gripper_action);
}

std::expected<void, std::string> ArmMotion::planAndRun(const char * what)
{
    if (stopped_.exchange(false))
    {
        return std::unexpected(std::format("{}: stopped", what));
    }
    group_.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode planned = group_.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return std::unexpected(
            std::format(
                "{}: planning failed ({})", what, moveit::core::errorCodeToString(planned)));
    }
    const moveit::core::MoveItErrorCode ran = group_.execute(plan);
    if (ran != moveit::core::MoveItErrorCode::SUCCESS)
    {
        return std::unexpected(
            std::format("{}: execution failed ({})", what, moveit::core::errorCodeToString(ran)));
    }
    return {};
}

std::expected<void, std::string> ArmMotion::toJoints(
    const JointValues & targets, double velocity_scale)
{
    group_.setMaxVelocityScalingFactor(
        velocity_scale > 0.0 ? velocity_scale : settings_.velocity_scale);
    group_.setMaxAccelerationScalingFactor(settings_.acceleration_scale);
    if (!group_.setJointValueTarget(targets))
    {
        return std::unexpected("joint targets outside the joints' limits");
    }
    group_.setPlanningPipelineId("pilz_industrial_motion_planner");
    group_.setPlannerId("PTP");
    auto ptp = planAndRun("PTP");
    if (ptp)
    {
        return ptp;
    }
    // PTP moves every joint at once and can sweep through an obstacle that OMPL goes round.
    group_.setPlanningPipelineId("ompl");
    group_.setPlannerId("");
    auto ompl = planAndRun("OMPL");
    if (!ompl)
    {
        return std::unexpected(std::format("{}; {}", ptp.error(), ompl.error()));
    }
    return ompl;
}

std::expected<void, std::string> ArmMotion::linear(
    const Eigen::Isometry3d & tcp, double velocity_scale)
{
    group_.setMaxVelocityScalingFactor(
        velocity_scale > 0.0 ? velocity_scale : settings_.velocity_scale);
    group_.setMaxAccelerationScalingFactor(settings_.acceleration_scale);
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = settings_.planning_frame;
    target.pose = tf2::toMsg(tcp);
    group_.setPoseTarget(target, settings_.tcp_link);
    group_.setPlanningPipelineId("pilz_industrial_motion_planner");
    group_.setPlannerId("LIN");
    auto lin = planAndRun("LIN");
    group_.clearPoseTargets();
    return lin;
}

std::expected<JointValues, std::string> ArmMotion::ik(const Eigen::Isometry3d & tcp)
{
    if (!ik_client_->wait_for_service(settings_.service_wait))
    {
        return std::unexpected("compute_ik is not offered: is move_group running?");
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto & ik = request->ik_request;
    ik.group_name = settings_.group;
    ik.ik_link_name = settings_.tcp_link;
    ik.avoid_collisions = true;
    ik.pose_stamped.header.frame_id = settings_.planning_frame;
    ik.pose_stamped.pose = tf2::toMsg(tcp);
    ik.timeout = rclcpp::Duration(settings_.ik_timeout);
    moveit::core::robotStateToRobotStateMsg(*group_.getCurrentState(), ik.robot_state);
    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(settings_.ik_timeout + settings_.service_wait) != std::future_status::ready)
    {
        ik_client_->remove_pending_request(future);
        return std::unexpected("compute_ik did not answer");
    }
    const auto response = future.get();
    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        return std::unexpected(
            std::format(
                "no IK solution ({})", moveit::core::errorCodeToString(response->error_code)));
    }
    const std::vector<std::string> & names = group_.getJointNames();
    JointValues joints;
    const auto & state = response->solution.joint_state;
    for (std::size_t i = 0; i < state.name.size() && i < state.position.size(); ++i)
    {
        if (std::ranges::find(names, state.name[i]) != names.end())
        {
            joints[state.name[i]] = state.position[i];
        }
    }
    return joints;
}

std::expected<double, std::string> ArmMotion::gripper(double position_m, double effort)
{
    using Command = control_msgs::action::ParallelGripperCommand;
    if (!gripper_client_->wait_for_action_server(settings_.service_wait))
    {
        return std::unexpected(std::format("{} is not offered", settings_.gripper_action));
    }
    Command::Goal goal;
    goal.command.name = {settings_.gripper_joint};
    goal.command.position = {position_m};
    goal.command.effort = {effort};
    auto accepted = gripper_client_->async_send_goal(goal);
    if (accepted.wait_for(settings_.service_wait) != std::future_status::ready || !accepted.get())
    {
        return std::unexpected("the gripper controller refused the goal");
    }
    const auto & handle = accepted.get();
    auto result = gripper_client_->async_get_result(handle);
    if (result.wait_for(settings_.gripper_timeout) != std::future_status::ready)
    {
        (void)gripper_client_->async_cancel_goal(handle);
        return std::unexpected("the gripper did not stop in time");
    }
    const auto & wrapped = result.get();
    const auto & state = wrapped.result->state;
    if (state.position.empty())
    {
        return std::unexpected("the gripper's result has no position");
    }
    // A stall on an object is how a grasp ends: it is not a failure.
    return state.position.front();
}

Eigen::Isometry3d ArmMotion::tcpPose()
{
    Eigen::Isometry3d pose;
    tf2::fromMsg(group_.getCurrentPose(settings_.tcp_link).pose, pose);
    return pose;
}

void ArmMotion::stop()
{
    stopped_.store(true);
    group_.stop();
}

void ArmMotion::clearStop()
{
    stopped_.store(false);
}

}  // namespace piper_skills
