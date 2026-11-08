#include "piper_bt/registration.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <string>

#include "piper_bt/ros_leaf.hpp"
#include "piper_bt/skills/find_objects.hpp"
#include "piper_bt/skills/move_arm_to_named_pose.hpp"
#include "piper_bt/skills/pick_object.hpp"
#include "piper_bt/skills/place_object.hpp"
#include "piper_bt/skills/place_target.hpp"
#include "piper_bt/skills/run_stage.hpp"
#include "piper_bt/skills/view_workspace.hpp"

namespace piper_bt
{

namespace
{

template <typename Leaf>
void registerLeaf(
    BT::BehaviorTreeFactory & factory, const std::string & id, const RosContext & context)
{
    factory.registerBuilder<Leaf>(
        id, [context](const std::string & name, const BT::NodeConfig & config) {
            return std::make_unique<Leaf>(name, config, context);
        });
}

}  // namespace

void registerLeaves(BT::BehaviorTreeFactory & factory, const RosContext & context)
{
    registerLeaf<MoveArmToNamedPose>(factory, "MoveArmToNamedPose", context);
    registerLeaf<ViewWorkspace>(factory, "ViewWorkspace", context);
    registerLeaf<FindObjects>(factory, "FindObjects", context);
    registerLeaf<PickObject>(factory, "PickObject", context);
    registerLeaf<PlaceObject>(factory, "PlaceObject", context);
    registerLeaf<PlaceTarget>(factory, "PlaceTarget", context);
    registerLeaf<RunStage>(factory, "RunStage", context);
}

std::string leafModelXml(const BT::BehaviorTreeFactory & factory)
{
    // Groot2 knows the built-in nodes.
    return BT::writeTreeNodesModelXML(factory, false);
}

}  // namespace piper_bt
