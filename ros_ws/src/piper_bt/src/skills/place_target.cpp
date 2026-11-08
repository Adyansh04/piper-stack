#include "piper_bt/skills/place_target.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "piper_bt/ros_leaf.hpp"
#include "piper_bt/ros_service_node.hpp"

namespace piper_bt
{

PlaceTarget::PlaceTarget(
    const std::string & name, const BT::NodeConfig & config, RosContext context)
    : RosServiceNode(name, config, std::move(context), "place_target")
{}

BT::PortsList PlaceTarget::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>(
            "mode", "free", "As for PlaceObject: free, relative, metric or anchor."),
        BT::InputPort<double>("x", 0.0, "As for PlaceObject."),
        BT::InputPort<double>("y", 0.0, "As for PlaceObject."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "anchor_pose",
            "none",
            "Anchor mode: where the anchor is, in the planning frame; none: no anchor."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
            "target", "Where the placed object's centre goes, in the planning frame."),
        BT::OutputPort<bool>("reachable", "Whether the arm reaches above the target."),
    });
}

bool PlaceTarget::fillRequest(Request & request)
{
    const auto mode = input<std::string>("mode");
    const auto x = input<double>("x");
    const auto y = input<double>("y");
    if (!mode || !x || !y)
    {
        return false;
    }
    request.mode = *mode;
    request.x = *x;
    request.y = *y;
    // Optional: none, the request's pose names no frame.
    const auto anchor_pose = optionalPose("anchor_pose");
    if (!anchor_pose)
    {
        return false;
    }
    request.anchor_pose = *anchor_pose;
    return true;
}

BT::NodeStatus PlaceTarget::judgeResponse(const Response & response)
{
    if (!response.success)
    {
        fail(response.message);
        return BT::NodeStatus::FAILURE;
    }
    output("target", response.target);
    output("reachable", static_cast<bool>(response.reachable));
    RCLCPP_INFO(
        logger(),
        "[%s] target %.2f, %.2f in %s, %s",
        name().c_str(),
        response.target.pose.position.x,
        response.target.pose.position.y,
        response.target.header.frame_id.c_str(),
        response.reachable ? "reachable" : "out of reach");
    return BT::NodeStatus::SUCCESS;
}

}  // namespace piper_bt
