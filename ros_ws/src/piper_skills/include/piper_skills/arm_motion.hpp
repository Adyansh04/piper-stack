#pragma once

#include <atomic>
#include <chrono>
#include <expected>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "piper_skills/srdf_states.hpp"

namespace piper_skills
{

/**
 * @brief How the arm and the gripper are moved.
 */
struct MotionSettings
{
    /**
     * @brief MoveIt's planning group, the link whose pose the skills set and the planning frame.
     */
    std::string group{"arm"};
    std::string tcp_link{"tcp"};
    std::string planning_frame{"world"};

    /**
     * @brief Default speed and acceleration scaling of the arm's moves, 0 to 1.
     */
    double velocity_scale{0.3};
    double acceleration_scale{0.3};

    /**
     * @brief Planning time per pipeline, seconds, and planning attempts.
     */
    double planning_time_s{5.0};
    int attempts{3};

    /**
     * @brief The gripper controller's action, and the finger joint it reports.
     */
    std::string gripper_action{"/gripper_controller/gripper_cmd"};
    std::string gripper_joint{"joint7"};

    /**
     * @brief Longest waits: for a gripper move, an IK answer, a service to appear.
     */
    std::chrono::milliseconds gripper_timeout{15000};
    std::chrono::milliseconds ik_timeout{1000};
    std::chrono::milliseconds service_wait{3000};
};

/**
 * @class ArmMotion
 * @brief Moves the arm through MoveIt and the gripper through its controller.
 * @details Joint goals are planned with Pilz PTP and, when that fails, OMPL; straight moves of the
 * TCP with Pilz LIN. IK goes through move_group's collision-aware `compute_ik`. Every call blocks
 * until done and must come from a thread that no executor of the node runs on.
 *
 * @note One thread moves at a time; stop() may come from any thread.
 */
class ArmMotion
{
public:
    /**
     * @brief Connects to MoveIt and the gripper.
     *
     * @param[in] node The node, owned by a shared pointer and spun by an executor elsewhere.
     * @param[in] settings Group, frames, scalings and timeouts.
     */
    ArmMotion(const rclcpp::Node::SharedPtr & node, MotionSettings settings);

    /**
     * @brief Moves the arm to joint positions.
     *
     * @param[in] targets Positions of the group's joints.
     * @param[in] velocity_scale Speed scaling, 0 to 1; 0 uses the default.
     *
     * @return Nothing, or why the move failed (planning, execution or a stop).
     */
    [[nodiscard]] std::expected<void, std::string> toJoints(
        const JointValues & targets, double velocity_scale = 0.0);

    /**
     * @brief Moves the TCP along a straight line to a pose.
     *
     * @param[in] tcp The TCP's pose in the planning frame.
     * @param[in] velocity_scale Speed scaling, 0 to 1; 0 uses the default.
     *
     * @return Nothing, or why the move failed.
     */
    [[nodiscard]] std::expected<void, std::string> linear(
        const Eigen::Isometry3d & tcp, double velocity_scale = 0.0);

    /**
     * @brief Joint positions that put the TCP at a pose without collisions.
     *
     * @param[in] tcp The TCP's pose in the planning frame.
     *
     * @return The group's joint positions, or why there are none.
     */
    [[nodiscard]] std::expected<JointValues, std::string> ik(const Eigen::Isometry3d & tcp);

    /**
     * @brief Moves the fingers and waits for them to stop.
     *
     * @param[in] position_m One finger's travel from closed, metres.
     * @param[in] effort Effort limit, in the controller's units.
     *
     * @return The finger's travel where it stopped, metres, or why the move failed.
     */
    [[nodiscard]] std::expected<double, std::string> gripper(double position_m, double effort);

    /**
     * @brief One finger's current travel from closed, from the robot's current state.
     * @return The travel in metres, or nullopt when no current state comes within a second.
     */
    [[nodiscard]] std::optional<double> fingerTravel();

    /**
     * @brief The TCP's current pose in the planning frame.
     */
    [[nodiscard]] Eigen::Isometry3d tcpPose();

    /**
     * @brief Stops a move in progress; the blocked call, and the next one, return a failure.
     */
    void stop();

    /**
     * @brief Forgets a stop that no move consumed, before a new skill starts.
     */
    void clearStop();

    /**
     * @brief The planning scene, for collision objects and attached loads.
     */
    [[nodiscard]] moveit::planning_interface::PlanningSceneInterface & scene()
    {
        return scene_;
    }

private:
    [[nodiscard]] std::expected<void, std::string> planAndRun(const char * what);

    rclcpp::Node::SharedPtr node_;
    MotionSettings settings_;
    moveit::planning_interface::MoveGroupInterface group_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr gripper_client_;
    std::atomic<bool> stopped_{false};
};

}  // namespace piper_skills
