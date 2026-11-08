#pragma once

#include <string>

#include <piper_msgs/action/pick_object.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class PickObject
 * @brief Picks an object off the table (`pick_object`): segments it in the current view,
 * looks again from close above, plans grasps (GraspGenX), grasps, lifts and checks it held.
 * @details Run FindObjects first so the camera sees the object. `min_score` is SAM 3.1's score
 * threshold (0: the skills' default); `approach` picks top-down grasps, side or diagonal ones, or
 * (auto) top grasps first and side grasps when none is reachable.
 */
class PickObject : public SkillActionNode<piper_msgs::action::PickObject>
{
public:
    /**
     * @brief The leaf, on the shared client of `pick_object`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    PickObject(const std::string & name, const BT::NodeConfig & config, RosContext context);

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
