#pragma once

#include <string>

#include <piper_msgs/action/move_arm_to_named_pose.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class MoveArmToNamedPose
 * @brief Moves the arm to one of the SRDF's named states, e.g. "ready" after a mission
 * (`move_arm_to_named_pose`).
 */
class MoveArmToNamedPose : public SkillActionNode<piper_msgs::action::MoveArmToNamedPose>
{
public:
    /**
     * @brief The leaf, on the shared client of `move_arm_to_named_pose`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    MoveArmToNamedPose(const std::string & name, const BT::NodeConfig & config, RosContext context);

    /**
     * @brief The leaf's ports.
     * @return Its own ports and those of every action leaf.
     */
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal & goal) override;
};

}  // namespace piper_bt
