#pragma once

#include <string>

#include <piper_msgs/action/find_objects.hpp>

#include "piper_bt/ros_action_node.hpp"

namespace piper_bt
{

/**
 * @class FindObjects
 * @brief Looks for objects matching a phrase on the table, view after view until found
 * (`find_objects`, SAM 3.1).
 * @details The camera stays at the view that found them, which is where PickObject starts.
 * Prompts that describe the look, specifically enough that nothing else matches, work best
 * ("t-shaped metal bracket" finds the T-connector, "t connector" does not).
 */
class FindObjects : public SkillActionNode<piper_msgs::action::FindObjects>
{
public:
    /**
     * @brief The leaf, on the shared client of `find_objects`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    FindObjects(const std::string & name, const BT::NodeConfig & config, RosContext context);

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
