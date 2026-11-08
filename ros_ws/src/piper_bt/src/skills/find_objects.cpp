#include "piper_bt/skills/find_objects.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

FindObjects::FindObjects(
    const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, std::move(context), "find_objects")
{}

BT::PortsList FindObjects::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("prompt", "What to look for, e.g. 'red cube'."),
        BT::InputPort<std::vector<std::string>>(
            "views", "", "Views to try in order, 'full;middle'; empty: the node's default."),
        BT::InputPort<double>(
            "min_score", 0.0, "Instances scoring below this are ignored (0: the default)."),
        BT::OutputPort<unsigned>("count", "How many were found."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
            "object_pose", "Pose of the best match, in the planning frame."),
        BT::OutputPort<std::string>("view", "The view that found them."),
        BT::OutputPort<bool>(
            "nothing_in_view",
            "Set true on a failure when the views the arm reaches were looked at to the end and "
            "show none; left alone otherwise."),
    });
}

bool FindObjects::fillGoal(Goal & goal)
{
    const auto prompt = input<std::string>("prompt");
    const auto views = input<std::vector<std::string>>("views");
    const auto min_score = input<double>("min_score");
    if (!prompt || !views || !min_score)
    {
        return false;
    }
    goal.prompt = *prompt;
    goal.views = *views;
    goal.min_score = static_cast<float>(*min_score);
    return true;
}

void FindObjects::onSuccess(const Result & result)
{
    output("count", static_cast<unsigned>(result.objects.size()));
    if (!result.objects.empty())
    {
        output("object_pose", result.objects.front().pose);
    }
    output("view", result.view);
}

void FindObjects::onFailure(const Result & result)
{
    if (result.nothing_in_view)
    {
        output("nothing_in_view", true);
    }
}

}  // namespace piper_bt
