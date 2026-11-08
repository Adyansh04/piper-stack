#include "piper_bt/skills/run_stage.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <string>
#include <utility>

#include "piper_bt/ros_action_node.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

RunStage::RunStage(const std::string & name, const BT::NodeConfig & config, RosContext context)
    : SkillActionNode(name, config, std::move(context), "run_stage")
{}

BT::PortsList RunStage::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("stage", "The stage's name in the task's stages, e.g. 'ring'."),
        BT::OutputPort<double>(
            "goal_error_m", "How far the stage's part ended from its goal, metres."),
    });
}

bool RunStage::fillGoal(Goal & goal)
{
    const auto stage = input<std::string>("stage");
    if (!stage)
    {
        return false;
    }
    goal.stage = *stage;
    return true;
}

void RunStage::onSuccess(const Result & result)
{
    output("goal_error_m", result.goal_error_m);
}

}  // namespace piper_bt
