#pragma once

#include <string>

#include <piper_msgs/srv/place_target.hpp>

#include "piper_bt/ros_service_node.hpp"

namespace piper_bt
{

/**
 * @class PlaceTarget
 * @brief Asks the arm skills where a place target lies on the table and whether the arm reaches
 * above it (`place_target`).
 * @details Takes PlaceObject's modes; outputs the target (in the planning frame) and `reachable`,
 * so a tree can choose another target before it picks.
 */
class PlaceTarget : public RosServiceNode<piper_msgs::srv::PlaceTarget>
{
public:
    /**
     * @brief The leaf, on the shared client of `place_target`.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    PlaceTarget(const std::string & name, const BT::NodeConfig & config, RosContext context);

    /**
     * @brief The leaf's ports.
     * @return Its own ports and those of every service leaf.
     */
    static BT::PortsList providedPorts();

protected:
    bool fillRequest(Request & request) override;
    BT::NodeStatus judgeResponse(const Response & response) override;
};

}  // namespace piper_bt
