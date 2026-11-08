#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace piper_skills
{

/**
 * @class SceneClient
 * @brief Reads and changes move_group's planning scene through its services, every call bounded
 * by a timeout.
 * @details Made with its node, well before the first call: a client made just before its first
 * request can miss move_group's reply, which a call without a timeout then waits for forever.
 *
 * @note A call blocks until answered or timed out: it must come from a thread that the callback
 * group's executor does not need to deliver the answer.
 */
class SceneClient
{
public:
    /**
     * @param[in] node The node that owns the clients.
     * @param[in] group The callback group that delivers move_group's answers.
     * @param[in] timeout The longest wait for move_group's answer to a call.
     */
    SceneClient(
        rclcpp::Node & node,
        const rclcpp::CallbackGroup::SharedPtr & group,
        std::chrono::milliseconds timeout);

    /**
     * @brief Applies a change to the scene.
     * @param[in] diff The change; sent as a diff.
     * @return Nothing, or why not: move_group is not up, did not answer or refused it.
     */
    [[nodiscard]] std::expected<void, std::string> apply(
        moveit_msgs::msg::PlanningScene diff) const;

    /**
     * @brief Parts of the scene.
     * @param[in] components The parts, as moveit_msgs/PlanningSceneComponents bits.
     * @return The scene with those parts, or why there is none.
     */
    [[nodiscard]] std::expected<moveit_msgs::msg::PlanningScene, std::string> get(
        std::uint32_t components) const;

    /**
     * @brief Where objects of the scene's world are, in the planning frame.
     * @param[in] ids The objects' ids.
     * @return Their poses by id, those not in the world (attached or unknown) left out.
     */
    [[nodiscard]] std::expected<std::map<std::string, Eigen::Isometry3d>, std::string> objectPoses(
        const std::vector<std::string> & ids) const;

private:
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_client_;
    std::chrono::milliseconds timeout_;
};

}  // namespace piper_skills
