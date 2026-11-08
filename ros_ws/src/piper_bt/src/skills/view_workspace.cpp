#include "piper_bt/skills/view_workspace.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

ViewWorkspace::ViewWorkspace(
    const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, std::move(context), "view_workspace")
{}

BT::PortsList ViewWorkspace::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("view", "full", "full, left, middle or right."),
    });
}

bool ViewWorkspace::fillGoal(Goal & goal)
{
    const auto view = input<std::string>("view");
    if (!view)
    {
        return false;
    }
    goal.view = *view;
    return true;
}

}  // namespace piper_bt
