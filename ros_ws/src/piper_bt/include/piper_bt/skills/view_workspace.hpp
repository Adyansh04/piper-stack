#pragma once

#include <string>

#include <piper_msgs/action/view_workspace.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class ViewWorkspace
 * @brief Points the wrist camera at the table (`view_workspace`): the "full" view, or a
 * section ("left", "middle", "right") the view planner made for this robot pose.
 */
class ViewWorkspace : public SkillActionNode<piper_msgs::action::ViewWorkspace>
{
public:
    /**
     * @brief The leaf, on the shared client of `view_workspace`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    ViewWorkspace(const std::string & name, const BT::NodeConfig & config, RosContext context);

    /**
     * @brief The leaf's ports.
     * @return Its own ports and those of every action leaf.
     */
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal & goal) override;
};

}  // namespace piper_bt
