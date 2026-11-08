#include "piper_bt/ros_leaf.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/tree_node.h>

#include <atomic>
#include <cmath>
#include <format>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace piper_bt
{

void reportFailure(
    const BT::TreeNode & node, const rclcpp::Logger & logger, const std::string & why)
{
    RCLCPP_ERROR(logger, "[%s] %s", node.name().c_str(), why.c_str());
    node.config().blackboard->rootBlackboard()->set<std::string>(
        "failure", std::format("{}: {}", node.name(), why));
}

void reportArmStuck(const BT::TreeNode & node, bool stuck)
{
    node.config().blackboard->rootBlackboard()->set<bool>("arm_stuck", stuck);
}

void reportHaltFailure(
    const BT::TreeNode & node, const rclcpp::Logger & logger, const std::string & why)
{
    RCLCPP_ERROR(logger, "[%s] %s", node.name().c_str(), why.c_str());
    node.config().blackboard->rootBlackboard()->set<std::string>(
        "halt_failure", std::format("{}: {}", node.name(), why));
}

std::string describeResultCode(rclcpp_action::ResultCode code)
{
    switch (code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return "the goal succeeded";
        case rclcpp_action::ResultCode::ABORTED:
            return "the server aborted the goal";
        case rclcpp_action::ResultCode::CANCELED:
            return "the goal was cancelled";
        case rclcpp_action::ResultCode::UNKNOWN:
            break;
    }
    return "the goal ended in an unknown state";
}

RosLeaf::RosLeaf(const std::string & name, const BT::NodeConfig & config, RosContext context)
    : BT::StatefulActionNode(name, config), context_(std::move(context))
{}

rclcpp::Logger RosLeaf::logger() const
{
    return context_.node->get_logger();
}

const RosContext & RosLeaf::context() const
{
    return context_;
}

bool RosLeaf::stopping() const
{
    return context_.stop != nullptr && context_.stop->load(std::memory_order_relaxed);
}

void RosLeaf::fail(const std::string & why) const
{
    reportFailure(*this, logger(), why);
}

std::optional<geometry_msgs::msg::PoseStamped> RosLeaf::optionalPose(const std::string & port)
{
    // BT.CPP parses a typed port's text with the port's own converter whatever type it is read
    // as, so "none" is looked for in the raw text, before any typed read.
    std::string text;
    if (const auto it = config().input_ports.find(port); it != config().input_ports.end())
    {
        text = it->second;
    }
    else if (config().manifest != nullptr)
    {
        if (const auto info = config().manifest->ports.find(port);
            info != config().manifest->ports.end())
        {
            text = info->second.defaultValueString();
        }
    }
    if (BT::StringView key; isBlackboardPointer(text, &key))
    {
        std::optional<std::string> entry_text;
        {
            // Released before the typed read below, which locks the entry again.
            const auto locked = config().blackboard->getAnyLocked(std::string(key));
            if (locked && locked.get()->isString())
            {
                entry_text = locked.get()->cast<std::string>();
            }
        }
        if (!entry_text)
        {
            // A pose entry (an earlier place_pose), or a missing one the read reports.
            return input<geometry_msgs::msg::PoseStamped>(port);
        }
        text = *entry_text;
    }
    if (text.empty() || text == "none")
    {
        return geometry_msgs::msg::PoseStamped{};
    }
    // Fails the leaf with the pose's own error when the text does not parse.
    return input<geometry_msgs::msg::PoseStamped>(port);
}

std::optional<double> RosLeaf::seconds(const std::string & port, double fallback_s)
{
    const auto value = input<double>(port);
    if (!value)
    {
        return std::nullopt;
    }
    if (!std::isfinite(*value) || *value < 0.0)
    {
        fail(std::format("port {} is {}: a time in seconds, 0 for the default", port, *value));
        return std::nullopt;
    }
    return *value > 0.0 ? *value : fallback_s;
}

}  // namespace piper_bt
