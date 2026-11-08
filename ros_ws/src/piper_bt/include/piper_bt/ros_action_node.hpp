#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <format>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "piper_bt/ros_leaf.hpp"

namespace piper_bt
{

/**
 * @class RosActionNode
 * @brief A leaf that drives one ROS action; no tick blocks except a halt.
 * @details The leaf stays RUNNING while it waits for the server (up to `server_timeout_s`) and
 * while the goal runs, and then returns the derived class's judgement of the result. It fails
 * when the server leaves the graph for `server_lost_s` or the goal runs longer than `timeout_s`,
 * which cancels it. Halting the leaf cancels its goal and waits up to `halt_wait_s` for the server
 * to confirm the stop, so a halted skill does not leave the arm or the base moving unnoticed: a
 * goal that does not confirm it is named in the run's result. The tree ticks on its own thread
 * while the node's executor runs the client callbacks; only the result crosses between them.
 *
 * @tparam ActionT The action type.
 */
template <typename ActionT>
class RosActionNode : public RosLeaf
{
public:
    using Goal = typename ActionT::Goal;
    using Result = typename ActionT::Result;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    /**
     * @brief A leaf of one action server.
     * @param name The leaf's name in the tree.
     * @param config Its ports and blackboard.
     * @param context The executor's node, clients, stop flag and settings.
     * @param action_name Action name; relative names resolve in the node's namespace.
     */
    RosActionNode(
        const std::string & name,
        const BT::NodeConfig & config,
        RosContext context,
        std::string action_name)
        : RosLeaf(name, config, std::move(context))
        , action_name_(std::move(action_name))
        , client_(this->context().clients->template action<ActionT>(action_name_))
    {}

    /**
     * @brief The ports every action leaf has.
     * @param ports The leaf's own ports.
     * @return They, `server_timeout_s` and `timeout_s`.
     */
    static BT::PortsList providedBasicPorts(BT::PortsList ports)
    {
        ports.insert(
            BT::InputPort<double>(
                "server_timeout_s",
                0.0,
                "How long to wait for the action server, seconds; 0: the executor's "
                "server_timeout_s."));
        ports.insert(
            BT::InputPort<double>(
                "timeout_s",
                0.0,
                "How long the goal may run before it is cancelled, seconds; 0: the executor's "
                "goal_timeout_s."));
        return ports;
    }

protected:
    /**
     * @brief Fills the goal from the ports.
     * @param goal The goal to fill.
     * @return False to fail the leaf without sending it (fail() with the reason first).
     */
    virtual bool fillGoal(Goal & goal) = 0;

    /**
     * @brief Turns the finished goal into the leaf's status.
     * @param result The goal's code and result.
     * @return SUCCESS or FAILURE.
     */
    virtual BT::NodeStatus judgeResult(const WrappedResult & result) = 0;

private:
    using Clock = std::chrono::steady_clock;

    // Shared with the client's result callback, which may run after the leaf is gone.
    struct State
    {
        /// Guards result and goals.
        std::mutex mutex;
        /// Notified when result is set.
        std::condition_variable done;
        std::optional<WrappedResult> result;
        /// Goals sent; only the latest one's result is kept.
        std::uint64_t goals{0};
    };

    BT::NodeStatus onStart() override
    {
        sent_ = false;
        if (stopping())
        {
            fail("the run was cancelled");
            return BT::NodeStatus::FAILURE;
        }
        const auto wait_s = seconds("server_timeout_s", context().server_timeout_s);
        const auto run_s = seconds("timeout_s", context().goal_timeout_s);
        if (!wait_s || !run_s)
        {
            return BT::NodeStatus::FAILURE;
        }
        wait_s_ = *wait_s;
        run_s_ = *run_s;
        server_deadline_ = Clock::now() + toDuration(wait_s_);
        if (finished_while_halted_)
        {
            const WrappedResult finished = *finished_while_halted_;
            finished_while_halted_.reset();
            return judgeResult(finished);
        }
        return sendWhenReady();
    }

    // RUNNING while the server is not there yet, so the tick loop sees a cancel meanwhile.
    BT::NodeStatus sendWhenReady()
    {
        if (!client_->action_server_is_ready())
        {
            if (Clock::now() < server_deadline_)
            {
                return BT::NodeStatus::RUNNING;
            }
            fail(std::format("no action server {} after {:.0f} s", action_name_, wait_s_));
            return BT::NodeStatus::FAILURE;
        }
        Goal goal;
        if (!fillGoal(goal))
        {
            return BT::NodeStatus::FAILURE;
        }
        std::uint64_t sent = 0;
        {
            const std::scoped_lock lock(state_->mutex);
            state_->result.reset();
            sent = ++state_->goals;
        }
        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        // Only this goal's result counts: a goal halted earlier may still answer late.
        options.result_callback = [state = state_, sent](const WrappedResult & result) {
            {
                const std::scoped_lock lock(state->mutex);
                if (sent != state->goals)
                {
                    return;
                }
                state->result = result;
            }
            state->done.notify_all();
        };
        // Accepted only after the leaf gave up on it (a stalled server): nothing else would cancel it.
        options.goal_response_callback =
            [state = state_, sent, client = client_](
                const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & handle) {
                bool dropped = false;
                {
                    const std::scoped_lock lock(state->mutex);
                    dropped = sent != state->goals;
                }
                if (handle && dropped)
                {
                    client->async_cancel_goal(handle);
                }
            };
        goal_future_ = client_->async_send_goal(goal, options);
        goal_deadline_ = Clock::now() + toDuration(run_s_);
        lost_since_.reset();
        sent_ = true;
        RCLCPP_INFO(logger(), "[%s] goal sent to %s", name().c_str(), action_name_.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!sent_)
        {
            if (stopping())
            {
                fail("the run was cancelled");
                return BT::NodeStatus::FAILURE;
            }
            return sendWhenReady();
        }
        std::optional<WrappedResult> result;
        {
            const std::scoped_lock lock(state_->mutex);
            result = state_->result;
        }
        if (result)
        {
            sent_ = false;
            return judgeResult(*result);
        }
        // A refused goal never gets a result.
        if (goal_future_.valid()
            && goal_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready
            && goal_future_.get() == nullptr)
        {
            sent_ = false;
            fail(std::format("{} refused the goal", action_name_));
            return BT::NodeStatus::FAILURE;
        }
        // A dead server sends no result; the graph shows it gone once the DDS lease expires.
        if (client_->action_server_is_ready())
        {
            lost_since_.reset();
        }
        else if (!lost_since_)
        {
            lost_since_ = Clock::now();
        }
        else if (Clock::now() - *lost_since_ > toDuration(context().server_lost_s))
        {
            forget();
            fail(std::format("the action server {} went away while its goal ran", action_name_));
            return BT::NodeStatus::FAILURE;
        }
        if (Clock::now() > goal_deadline_)
        {
            const bool stopped = stopGoal();
            fail(
                std::format(
                    "{} did not finish within {:.0f} s and was cancelled{}",
                    action_name_,
                    run_s_,
                    stopped ? "" : ", but did not confirm the stop"));
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // The step already ended, between two ticks (a Pause lets it): Resume must not run it again.
        if (sent_)
        {
            const std::scoped_lock lock(state_->mutex);
            finished_while_halted_ = state_->result;
        }
        if (sent_ && !stopGoal())
        {
            reportHaltFailure(
                *this,
                logger(),
                std::format(
                    "{} did not confirm the cancel within {:.0f} s and may still be running",
                    action_name_,
                    context().halt_wait_s));
        }
    }

    // Cancels the goal and waits for its result; false when the server did not answer in time.
    // Never throws: a tree halts from its destructor too.
    bool stopGoal()
    {
        sent_ = false;
        // At the shutdown nothing answers any more: cancel without waiting.
        const auto wait =
            rclcpp::ok() ? toDuration(context().halt_wait_s) : Clock::duration::zero();
        bool stopped = true;
        try
        {
            // A goal still on its way can be accepted after the halt.
            if (goal_future_.valid() && goal_future_.wait_for(wait) == std::future_status::ready)
            {
                if (const auto handle = goal_future_.get())
                {
                    std::unique_lock lock(state_->mutex);
                    if (!state_->result)
                    {
                        lock.unlock();
                        client_->async_cancel_goal(handle);
                        RCLCPP_INFO(logger(), "[%s] goal cancelled", name().c_str());
                        lock.lock();
                        stopped = !rclcpp::ok() || state_->done.wait_for(lock, wait, [this] {
                            return state_->result.has_value();
                        });
                    }
                }
            }
            else if (goal_future_.valid())
            {
                stopped = false;
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_WARN(logger(), "[%s] cancelling the goal failed: %s", name().c_str(), e.what());
            stopped = false;
        }
        forget();
        return stopped;
    }

    // Drops the goal: a result that comes later is ignored.
    void forget()
    {
        sent_ = false;
        const std::scoped_lock lock(state_->mutex);
        state_->result.reset();
        ++state_->goals;
    }

    static Clock::duration toDuration(double seconds)
    {
        return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    }

    std::string action_name_;
    std::shared_ptr<rclcpp_action::Client<ActionT>> client_;
    std::shared_ptr<State> state_{std::make_shared<State>()};
    std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
    // The tick thread's alone.
    bool sent_{false};
    double wait_s_{0.0};
    double run_s_{0.0};
    Clock::time_point server_deadline_;
    Clock::time_point goal_deadline_;
    std::optional<Clock::time_point> lost_since_;
    // A result that was in when a Hold halted the leaf: the step is not run again on Resume.
    std::optional<WrappedResult> finished_while_halted_;
};

/**
 * @class SkillActionNode
 * @brief A leaf for an action whose result carries `success` and `message`, as every piper
 * skill server's does.
 * @details A goal that did not succeed, or a result with `success` false, fails the leaf with the
 * server's message; a derived leaf only fills the goal and, if it has outputs, writes them in
 * onSuccess().
 *
 * @tparam ActionT The action type.
 */
template <typename ActionT>
class SkillActionNode : public RosActionNode<ActionT>
{
public:
    using RosActionNode<ActionT>::RosActionNode;
    using typename RosActionNode<ActionT>::Result;
    using typename RosActionNode<ActionT>::WrappedResult;

protected:
    /**
     * @brief Writes the result's outputs to the ports.
     * @param result The successful result.
     */
    virtual void onSuccess(const Result & /*result*/) {}

    /**
     * @brief Writes what a failed result still tells to the ports.
     * @param result The result of a goal that ran to its end.
     */
    virtual void onFailure(const Result & /*result*/) {}

    BT::NodeStatus judgeResult(const WrappedResult & result) override
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result
            || !result.result->success)
        {
            if (result.result)
            {
                onFailure(*result.result);
            }
            this->fail(
                result.result && !result.result->message.empty() ? result.result->message
                                                                 : describeResultCode(result.code));
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(
            this->logger(), "[%s] %s", this->name().c_str(), result.result->message.c_str());
        onSuccess(*result.result);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace piper_bt
