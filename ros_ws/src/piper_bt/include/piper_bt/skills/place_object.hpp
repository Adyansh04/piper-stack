#pragma once

#include <string>

#include <piper_msgs/action/place_object.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class PlaceObject
 * @brief Sets the held object down on a free spot of the table (`place_object`).
 * @details Fails at once when the gripper holds nothing. Outputs where the object went, in the
 * planning frame.
 */
class PlaceObject : public SkillActionNode<piper_msgs::action::PlaceObject>
{
public:
    /**
     * @brief The leaf, on the shared client of `place_object`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    PlaceObject(const std::string & name, const BT::NodeConfig & config, RosContext context);

    /**
     * @brief The leaf's ports.
     * @return Its own ports and those of every action leaf.
     */
    static BT::PortsList providedPorts();

protected:
    bool fillGoal(Goal & goal) override;
    void onSuccess(const Result & result) override;
    void onFailure(const Result & result) override;
};

}  // namespace piper_bt
