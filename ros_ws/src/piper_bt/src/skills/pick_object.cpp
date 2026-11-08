#include "piper_bt/skills/pick_object.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

PickObject::PickObject(const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, std::move(context), "pick_object")
{}

BT::PortsList PickObject::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("prompt", "What to pick, e.g. 'red cube'."),
        BT::InputPort<double>(
            "min_score", 0.0, "Instances scoring below this are ignored (0: the default)."),
        BT::InputPort<std::string>(
            "approach",
            "auto",
            "Which grasps: top, side (from the side or diagonally), or auto (top first, side when "
            "no top grasp is reachable)."),
        BT::OutputPort<double>("finger_width_m", "Finger opening on the held object, metres."),
    });
}

bool PickObject::fillGoal(Goal & goal)
{
    const auto prompt = input<std::string>("prompt");
    const auto min_score = input<double>("min_score");
    const auto approach = input<std::string>("approach");
    if (!prompt || !min_score || !approach)
    {
        return false;
    }
    goal.prompt = *prompt;
    goal.min_score = static_cast<float>(*min_score);
    goal.approach = *approach;
    return true;
}

void PickObject::onFailure(const Result & result)
{
    if (result.arm_stuck)
    {
        reportArmStuck(*this, true);
    }
}

void PickObject::onSuccess(const Result & result)
{
    reportArmStuck(*this, false);
    output("finger_width_m", result.finger_width_m);
}

}  // namespace piper_bt
