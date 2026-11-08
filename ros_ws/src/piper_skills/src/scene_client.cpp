#include "piper_skills/scene_client.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <expected>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace piper_skills
{

SceneClient::SceneClient(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & group,
    std::chrono::milliseconds timeout)
    : apply_client_(node.create_client<moveit_msgs::srv::ApplyPlanningScene>(
          "/apply_planning_scene", rclcpp::ServicesQoS(), group))
    , get_client_(node.create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene", rclcpp::ServicesQoS(), group))
    , timeout_(timeout)
{}

std::expected<void, std::string> SceneClient::apply(moveit_msgs::msg::PlanningScene diff) const
{
    if (!apply_client_->service_is_ready())
    {
        return std::unexpected("/apply_planning_scene is not offered: is move_group running?");
    }
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = std::move(diff);
    request->scene.is_diff = true;
    auto reply = apply_client_->async_send_request(request);
    if (reply.wait_for(timeout_) != std::future_status::ready)
    {
        apply_client_->remove_pending_request(reply);
        return std::unexpected("move_group did not answer /apply_planning_scene");
    }
    if (!reply.get()->success)
    {
        return std::unexpected("move_group refused the change to its scene");
    }
    return {};
}

std::expected<moveit_msgs::msg::PlanningScene, std::string> SceneClient::get(
    std::uint32_t components) const
{
    if (!get_client_->service_is_ready())
    {
        return std::unexpected("/get_planning_scene is not offered: is move_group running?");
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = components;
    auto reply = get_client_->async_send_request(request);
    if (reply.wait_for(timeout_) != std::future_status::ready)
    {
        get_client_->remove_pending_request(reply);
        return std::unexpected("move_group did not answer /get_planning_scene");
    }
    return reply.get()->scene;
}

std::expected<std::map<std::string, Eigen::Isometry3d>, std::string> SceneClient::objectPoses(
    const std::vector<std::string> & ids) const
{
    // Only the objects' geometry carries their poses.
    const auto scene = get(moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY);
    if (!scene)
    {
        return std::unexpected(scene.error());
    }
    std::map<std::string, Eigen::Isometry3d> poses;
    for (const auto & object : scene->world.collision_objects)
    {
        if (std::ranges::find(ids, object.id) != ids.end())
        {
            tf2::fromMsg(object.pose, poses[object.id]);
        }
    }
    return poses;
}

}  // namespace piper_skills
