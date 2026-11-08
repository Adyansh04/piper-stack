#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

/**
 * @brief Registers every piper leaf under the name the trees use for it.
 * @param factory The factory.
 * @param context What the leaves take from the executor; copied into each leaf.
 */
void registerLeaves(BT::BehaviorTreeFactory & factory, const RosContext & context);

/**
 * @brief The TreeNodesModel of the piper leaves, the palette Groot2 edits trees with.
 * @param factory A factory with the leaves registered.
 * @return XML text.
 */
[[nodiscard]] std::string leafModelXml(const BT::BehaviorTreeFactory & factory);

}  // namespace piper_bt
