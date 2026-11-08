#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <chrono>
#include <format>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

/**
 * @class RosServiceNode
 * @brief A leaf that calls one ROS service without blocking a tick.
 * @details The leaf stays RUNNING while it waits for the service (up to `server_timeout_s`) and
 * for the response, and fails after `response_timeout_s`. Halting it drops the pending request.
 *
 * @tparam ServiceT The service type.
 */
template <typename ServiceT>
class RosServiceNode : public RosLeaf
{
public:
    using Request = typename ServiceT::Request;
    using Response = typename ServiceT::Response;

    /**
     * @brief A leaf of one service.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     * @param service_name Service name; relative names resolve in the node's namespace.
     */
    RosServiceNode(
        const std::string & name,
        const BT::NodeConfig & config,
        RosContext context,
        std::string service_name)
        : RosLeaf(name, config, std::move(context))
        , service_name_(std::move(service_name))
        , client_(this->context().clients->template service<ServiceT>(service_name_))
    {}

    /**
     * @brief The ports every service leaf has.
     * @param ports The leaf's own ports.
     * @return They, `server_timeout_s` and `response_timeout_s`.
     */
    static BT::PortsList providedBasicPorts(BT::PortsList ports)
    {
        ports.insert(
            BT::InputPort<double>(
                "server_timeout_s",
                0.0,
                "How long to wait for the service, seconds; 0: the executor's server_timeout_s."));
        ports.insert(
            BT::InputPort<double>(
                "response_timeout_s",
                0.0,
                "How long to wait for the response, seconds; 0: the executor's "
                "response_timeout_s."));
        return ports;
    }

protected:
    /**
     * @brief Fills the request from the ports.
     * @param request The request to fill.
     * @return False to fail the leaf without calling (fail() with the reason first).
     */
    virtual bool fillRequest(Request & request) = 0;

    /**
     * @brief Turns the response into the leaf's status and writes its outputs.
     * @param response The service's response.
     * @return SUCCESS or FAILURE.
     */
    virtual BT::NodeStatus judgeResponse(const Response & response) = 0;

private:
    using Clock = std::chrono::steady_clock;

    BT::NodeStatus onStart() override
    {
        pending_.reset();
        if (stopping())
        {
            fail("the run was cancelled");
            return BT::NodeStatus::FAILURE;
        }
        const auto wait_s = seconds("server_timeout_s", context().server_timeout_s);
        const auto answer_s = seconds("response_timeout_s", context().response_timeout_s);
        if (!wait_s || !answer_s)
        {
            return BT::NodeStatus::FAILURE;
        }
        wait_s_ = *wait_s;
        answer_s_ = *answer_s;
        deadline_ = Clock::now() + toDuration(wait_s_);
        return sendWhenReady();
    }

    // RUNNING while the service is not there yet, so the tick loop sees a cancel meanwhile.
    BT::NodeStatus sendWhenReady()
    {
        if (!client_->service_is_ready())
        {
            if (Clock::now() < deadline_)
            {
                return BT::NodeStatus::RUNNING;
            }
            fail(std::format("no service {} after {:.0f} s", service_name_, wait_s_));
            return BT::NodeStatus::FAILURE;
        }
        auto request = std::make_shared<Request>();
        if (!fillRequest(*request))
        {
            return BT::NodeStatus::FAILURE;
        }
        deadline_ = Clock::now() + toDuration(answer_s_);
        pending_.emplace(client_->async_send_request(request));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!pending_)
        {
            if (stopping())
            {
                fail("the run was cancelled");
                return BT::NodeStatus::FAILURE;
            }
            return sendWhenReady();
        }
        if (pending_->wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        {
            if (Clock::now() < deadline_)
            {
                return BT::NodeStatus::RUNNING;
            }
            client_->remove_pending_request(pending_->request_id);
            pending_.reset();
            fail(std::format("{} did not answer within {:.0f} s", service_name_, answer_s_));
            return BT::NodeStatus::FAILURE;
        }
        const auto response = pending_->get();
        pending_.reset();
        return judgeResponse(*response);
    }

    void onHalted() override
    {
        if (pending_)
        {
            client_->remove_pending_request(pending_->request_id);
            pending_.reset();
        }
    }

    static Clock::duration toDuration(double seconds)
    {
        return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    }

    std::string service_name_;
    std::shared_ptr<rclcpp::Client<ServiceT>> client_;
    // The tick thread's alone.
    std::optional<typename rclcpp::Client<ServiceT>::FutureAndRequestId> pending_;
    double wait_s_{0.0};
    double answer_s_{0.0};
    Clock::time_point deadline_;
};

}  // namespace piper_bt
