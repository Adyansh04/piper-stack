#pragma once

#include <string>

#include <piper_msgs/action/run_stage.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class RunStage
 * @brief Runs one stage of the contact task the skills were started with (`run_stage`): grasps
 * its part, puts it on its mate, screws or twists it home and checks where it ended.
 */
class RunStage : public SkillActionNode<piper_msgs::action::RunStage>
{
public:
    /**
     * @brief The leaf, on the shared client of `run_stage`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    RunStage(const std::string & name, const BT::NodeConfig & config, RosContext context);

    /**
     * @brief The leaf's ports.
     * @return Its own ports and those of every action leaf.
     */
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal & goal) override;
    void onSuccess(const Result & result) override;
};

}  // namespace piper_bt
