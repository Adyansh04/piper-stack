#include "piper_bt/skills/place_object.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <cmath>
#include <format>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

PlaceObject::PlaceObject(
    const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, std::move(context), "place_object")
{}

BT::PortsList PlaceObject::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>(
            "region",
            "any",
            "Free mode: any, left, middle or right (thirds of the work area as the robot sees "
            "it)."),
        BT::InputPort<std::string>(
            "mode",
            "free",
            "free (the free spot nearest the robot in the region), relative (x and y from -1 to 1 "
            "across the work area), metric (metres from its centre) or anchor (metres from the "
            "anchor)."),
        BT::InputPort<double>("x", 0.0, "To the robot's right: -1 to 1 or metres."),
        BT::InputPort<double>("y", 0.0, "Away from the robot: -1 to 1 or metres."),
        BT::InputPort<double>(
            "tolerance_m",
            0.0,
            "How far from the target the object may go when it is taken, metres."),
        BT::InputPort<std::string>(
            "yaw_deg",
            "any",
            "The object's long axis from the table's x axis (away from the robot), degrees; any: "
            "any heading."),
        BT::InputPort<double>("yaw_tolerance_deg", 0.0, "How far the heading may be off, degrees."),
        BT::InputPort<std::string>(
            "anchor", "none", "Anchor mode: what to find on the table, e.g. 'red cube'."),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "anchor_pose",
            "none",
            "Anchor mode: where the anchor is, in the planning frame (e.g. an earlier place_pose); "
            "none: the best match on the table."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
            "place_pose", "Where the object went, in the planning frame."),
    });
}

bool PlaceObject::fillGoal(Goal & goal)
{
    const auto region = input<std::string>("region");
    const auto mode = input<std::string>("mode");
    const auto x = input<double>("x");
    const auto y = input<double>("y");
    const auto tolerance_m = input<double>("tolerance_m");
    const auto yaw_deg = input<std::string>("yaw_deg");
    const auto yaw_tolerance_deg = input<double>("yaw_tolerance_deg");
    const auto anchor = input<std::string>("anchor");
    if (!region || !mode || !x || !y || !tolerance_m || !yaw_deg || !yaw_tolerance_deg || !anchor)
    {
        return false;
    }
    goal.region = *region;
    goal.mode = *mode;
    goal.x = *x;
    goal.y = *y;
    goal.tolerance_m = *tolerance_m;
    goal.use_yaw = !yaw_deg->empty() && *yaw_deg != "any";
    if (goal.use_yaw)
    {
        double value = 0.0;
        if (!BT::parseDouble(*yaw_deg, value, true) || !std::isfinite(value))
        {
            fail(std::format("yaw_deg '{}' is not a number", *yaw_deg));
            return false;
        }
        goal.yaw_deg = value;
    }
    goal.yaw_tolerance_deg = *yaw_tolerance_deg;
    goal.anchor = *anchor == "none" ? std::string{} : *anchor;
    // Optional: none, the goal's pose names no frame.
    const auto anchor_pose = optionalPose("anchor_pose");
    if (!anchor_pose)
    {
        return false;
    }
    goal.anchor_pose = *anchor_pose;
    return true;
}

void PlaceObject::onFailure(const Result & result)
{
    if (result.arm_stuck)
    {
        reportArmStuck(*this, true);
    }
}

void PlaceObject::onSuccess(const Result & result)
{
    reportArmStuck(*this, false);
    output("place_pose", result.place_pose);
}

}  // namespace piper_bt
