// Prints the piper leaves' TreeNodesModel, the palette Groot2 edits the trees with:
// ros2 run piper_bt bt_node_model > groot/piper_bt_nodes.xml
#include <behaviortree_cpp/bt_factory.h>

#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "piper_bt/registration.hpp"
#include "piper_bt/ros_leaf.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("bt_node_model");
    piper_bt::RosContext context;
    context.node = node.get();
    context.clients = std::make_shared<piper_bt::Clients>(node.get());
    BT::BehaviorTreeFactory factory;
    piper_bt::registerLeaves(factory, context);
    std::cout << piper_bt::leafModelXml(factory) << '\n';
    rclcpp::shutdown();
    return 0;
}
