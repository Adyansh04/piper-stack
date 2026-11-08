#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/tree_node.h>

#include <atomic>
#include <format>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <typeinfo>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Every leaf sees the PoseStamped conversion, so no port can instantiate the throwing primary
// template instead.
#include "piper_bt/port_types.hpp"

namespace piper_bt
{

/**
 * @class Clients
 * @brief The action and service clients of every leaf of every run, one per server name.
 * @details Leaves are made anew for each run; their clients stay, so a run does not wait for new
 * endpoints to be discovered. Callbacks are per goal and per request, so leaves share a client
 * safely. Thread-safe.
 */
class Clients
{
public:
    /**
     * @brief An empty set of clients on a node.
     * @param node The node the clients live on; not owned, it must outlive them.
     */
    explicit Clients(rclcpp::Node * node) : node_(node) {}

    /**
     * @brief The client of an action server.
     * @tparam ActionT The action type.
     * @param name The server's name; relative names resolve in the node's namespace.
     * @return The client, made on first use.
     */
    template <typename ActionT>
    [[nodiscard]] std::shared_ptr<rclcpp_action::Client<ActionT>> action(const std::string & name)
    {
        return get<rclcpp_action::Client<ActionT>>(
            name, [&] { return rclcpp_action::create_client<ActionT>(node_, name); });
    }

    /**
     * @brief The client of a service.
     * @tparam ServiceT The service type.
     * @param name The service's name; relative names resolve in the node's namespace.
     * @return The client, made on first use.
     */
    template <typename ServiceT>
    [[nodiscard]] std::shared_ptr<rclcpp::Client<ServiceT>> service(const std::string & name)
    {
        return get<rclcpp::Client<ServiceT>>(
            name, [&] { return node_->create_client<ServiceT>(name); });
    }

private:
    template <typename ClientT, typename Make>
    std::shared_ptr<ClientT> get(const std::string & name, const Make & make)
    {
        const std::scoped_lock lock(mutex_);
        auto & slot = clients_[std::string(typeid(ClientT).name()) + "/" + name];
        if (!slot)
        {
            slot = make();
        }
        return std::static_pointer_cast<ClientT>(slot);
    }

    rclcpp::Node * node_;
    /// Guards clients_.
    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<void>> clients_;
};

/** @brief What every leaf takes from the executor. */
struct RosContext
{
    /// The executor's node: the leaves log through it. Not owned: the node outlives every tree.
    rclcpp::Node * node{nullptr};
    /// The clients the leaves share.
    std::shared_ptr<Clients> clients;
    /// Set once the running tree is to stop (a cancel or the shutdown): a leaf about to start
    /// something fails at once instead. Read relaxed: it only gates new work.
    const std::atomic<bool> * stop{nullptr};
    /// The leaves' time limits where a tree sets none, seconds (bt_executor.yaml): waiting for a
    /// server, for a service's response, for a goal's result (a long one, and a short one for the
    /// goals of withShortGoals), for a server that left the graph to come back, and for a halted
    /// goal to confirm its stop.
    double server_timeout_s{10.0};
    double response_timeout_s{30.0};
    double goal_timeout_s{900.0};
    double short_goal_timeout_s{180.0};
    double server_lost_s{10.0};
    double halt_wait_s{10.0};
};

/**
 * @brief The context for a leaf whose goals end in a minute or two.
 * @param context The executor's context.
 * @return It with short_goal_timeout_s as the goal limit.
 */
[[nodiscard]] inline RosContext withShortGoals(RosContext context)
{
    context.goal_timeout_s = context.short_goal_timeout_s;
    return context;
}

/**
 * @brief Records whether the arm is still down at a top or an object.
 * @details Goes to the root blackboard's `arm_stuck` entry, which the executor sets false at the
 * start of a run. While it is true the library trees move neither the arm nor the base.
 *
 * @param node The leaf that knows.
 * @param stuck Whether the arm is stuck.
 */
void reportArmStuck(const BT::TreeNode & node, bool stuck);

/**
 * @brief Records why a leaf failed, for the run's result, and logs it.
 * @details The reason goes to the root blackboard's `failure` entry as "<leaf>: <why>", so the
 * last failure of a run names the step and its cause.
 *
 * @param node The failing leaf.
 * @param logger Where to log.
 * @param why The reason, in words an operator can act on.
 */
void reportFailure(
    const BT::TreeNode & node, const rclcpp::Logger & logger, const std::string & why);

/**
 * @brief Records that a halted leaf's goal did not confirm its stop, for the run's result.
 * @details The note goes to the root blackboard's `halt_failure` entry as "<leaf>: <why>".
 *
 * @param node The halted leaf.
 * @param logger Where to log.
 * @param why What may still be moving.
 */
void reportHaltFailure(
    const BT::TreeNode & node, const rclcpp::Logger & logger, const std::string & why);

/**
 * @brief How a finished goal ended, in words.
 * @param code The goal's result code.
 * @return A short phrase, e.g. "the server aborted the goal".
 */
[[nodiscard]] std::string describeResultCode(rclcpp_action::ResultCode code);

/**
 * @class RosLeaf
 * @brief What the action and service leaves share: the executor's context, failure reports and
 * the reading of ports.
 */
class RosLeaf : public BT::StatefulActionNode
{
public:
    /**
     * @brief A leaf in a tree.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     */
    RosLeaf(const std::string & name, const BT::NodeConfig & config, RosContext context);

protected:
    /** @brief The executor's logger. */
    [[nodiscard]] rclcpp::Logger logger() const;

    /** @brief The executor's node, clients, stop flag, TF buffer, frames and time limits. */
    [[nodiscard]] const RosContext & context() const;

    /** @brief Whether the run is being stopped. */
    [[nodiscard]] bool stopping() const;

    /**
     * @brief Fails the leaf with a reason (reportFailure()).
     * @param why The reason, in words an operator can act on.
     */
    void fail(const std::string & why) const;

    /**
     * @brief A port's value, or nothing after failing the leaf with the reason.
     * @details An attribute left out gives the port's default; a value that does not parse, a
     * required port left out and a blackboard entry that is not set fail the leaf.
     * @tparam T The port's type.
     * @param port The port's name.
     * @return The value; nullopt when the leaf failed.
     */
    template <typename T>
    [[nodiscard]] std::optional<T> input(const std::string & port)
    {
        auto value = getInput<T>(port);
        if (!value)
        {
            fail(std::format("port {}: {}", port, value.error()));
            return std::nullopt;
        }
        return std::move(*value);
    }

    /**
     * @brief Writes an output port the tree remaps; one the tree leaves out is skipped.
     * @details A value the port's entry does not take (another type) is logged as an error.
     * @tparam T The value's type.
     * @param port The port's name.
     * @param value The value to write.
     */
    template <typename T>
    void output(const std::string & port, const T & value)
    {
        if (!config().output_ports.contains(port))
        {
            return;
        }
        if (const auto written = setOutput(port, value); !written)
        {
            RCLCPP_ERROR(
                logger(),
                "[%s] port %s: %s",
                name().c_str(),
                port.c_str(),
                written.error().c_str());
        }
    }

    /**
     * @brief A time limit port, seconds: its value, or `fallback_s` where it is 0.
     * @param port The port's name.
     * @param fallback_s The executor's default, seconds.
     * @return The limit; nullopt after failing the leaf when the value is negative or unusable.
     */
    [[nodiscard]] std::optional<double> seconds(const std::string & port, double fallback_s);

    /**
     * @brief An optional pose port's value.
     * @details "none" or empty text (a default or a blackboard entry holding it) gives a pose whose
     * header names no frame; a pose entry or its text ("x;y;yaw;frame") gives that pose.
     * @param port The port's name.
     * @return The pose; nullopt after failing the leaf when the value does not parse.
     */
    [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> optionalPose(
        const std::string & port);

private:
    RosContext context_;
};

}  // namespace piper_bt
