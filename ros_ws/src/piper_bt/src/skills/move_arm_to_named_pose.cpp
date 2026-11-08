#include "piper_bt/skills/move_arm_to_named_pose.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

MoveArmToNamedPose::MoveArmToNamedPose(
    const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, withShortGoals(std::move(context)), "move_arm_to_named_pose")
{}

BT::PortsList MoveArmToNamedPose::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>(
            "pose", "ready", "The SRDF state: ready, rest, view_left or view_right."),
        BT::InputPort<double>(
            "velocity_scale", 0.0, "Share of the joints' top speed, 0 to 1 (0: the default)."),
    });
}

bool MoveArmToNamedPose::fillGoal(Goal & goal)
{
    const auto pose = input<std::string>("pose");
    const auto velocity_scale = input<double>("velocity_scale");
    if (!pose || !velocity_scale)
    {
        return false;
    }
    goal.name = *pose;
    goal.velocity_scale = *velocity_scale;
    return true;
}

}  // namespace piper_bt
