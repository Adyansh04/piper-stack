#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <piper_msgs/action/run_tree.hpp>
#include <piper_msgs/msg/blackboard_entry.hpp>
#include <piper_msgs/msg/tree_node_status.hpp>
#include <piper_msgs/msg/tree_status.hpp>
#include <piper_msgs/srv/control_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "piper_bt/ros_leaf.hpp"
#include "piper_bt/tree_library.hpp"

namespace piper_bt
{

/**
 * @brief Splits "key=value" blackboard entries.
 * @param entries The entries.
 * @return The pairs, in order.
 * @throws std::invalid_argument If an entry has no '=' or an empty key.
 */
[[nodiscard]] std::vector<std::pair<std::string, std::string>> parseBlackboard(
    const std::vector<std::string> & entries);

/**
 * @brief The blackboard of a run, checked against its tree's inputs.
 * @param entries The goal's "key=value" entries, already split.
 * @param inputs The tree's declared inputs; nullopt when it declares none, and then any key goes.
 * @return The entries with the defaults of inputs left out added.
 * @throws std::invalid_argument If a key is not an input of the tree or a needed one is missing.
 */
[[nodiscard]] std::vector<std::pair<std::string, std::string>> checkBlackboard(
    std::vector<std::pair<std::string, std::string>> entries,
    const std::optional<TreeInputs> & inputs);

/**
 * @class BtExecutor
 * @brief Runs piper's behaviour trees: one tree at a time, to its end, through the `run_tree`
 * action.
 * @details It registers the leaves and every tree file under `tree_directories` (each
 * `<BehaviorTree>` by its ID, so trees in one file can use subtrees from another). A goal names a
 * tree ID and "key=value" blackboard entries, checked against the tree's declared inputs; the tree
 * ticks at `tick_rate_hz` on a thread of its own, the feedback lists the leaves running, and
 * cancelling halts the tree, which cancels the running skill's goal and waits for it to stop. A
 * goal sent while a tree runs, or while a halted one stops, is rejected. The result's message is
 * the last failing leaf's reason. Groot2 can watch a run on `groot2_port`.
 *
 * For clients that show or change runs: `~/status` (TreeStatus: the running tree's nodes and
 * blackboard) and `~/control` (pause, hold, resume). For monitors: `diagnostics` (DiagnosticArray
 * with one status, "<node>: mission": the tree, its state, the running leaves and the last
 * result).
 */
class BtExecutor : public rclcpp::Node
{
public:
    using RunTree = piper_msgs::action::RunTree;
    using GoalHandle = rclcpp_action::ServerGoalHandle<RunTree>;
    using TreeStatus = piper_msgs::msg::TreeStatus;

    /**
     * @brief Reads the parameters, registers the leaves, loads the trees and starts the action
     * server.
     * @param options Node options.
     * @return The executor.
     * @throws std::invalid_argument If a parameter is out of range.
     * @throws std::runtime_error If a tree file cannot be read or does not load, two trees share an
     * ID, or no tree is found.
     * @throws std::filesystem::filesystem_error If a tree directory cannot be read.
     * @throws ament_index_cpp::PackageNotFoundError If a "package://" directory names a package
     * that is not installed.
     */
    [[nodiscard]] static std::shared_ptr<BtExecutor> create(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /** @brief Waits for a running tree's thread to end (finish()). */
    ~BtExecutor() override;

    BtExecutor(const BtExecutor &) = delete;
    BtExecutor & operator=(const BtExecutor &) = delete;
    BtExecutor(BtExecutor &&) = delete;
    BtExecutor & operator=(BtExecutor &&) = delete;

    /** @brief Halts a running tree and waits for its thread; call before shutting down. */
    void finish();

    /** @brief The tree IDs that can be run. */
    [[nodiscard]] std::vector<std::string> trees() const;

    /** @brief How many threads the node's executor should spin with (`executor_threads`). */
    [[nodiscard]] int executorThreads() const;

private:
    // How a run ended.
    enum class Outcome : std::uint8_t
    {
        kSucceeded,
        kFailed,
        kHalted
    };

    // Each node's last status other than IDLE in a run (BT.CPP resets finished nodes to IDLE).
    class LastStatus;

    // What the tick loop remembers between ticks: the last feedback and statuses sent, and whether
    // a Hold has halted the running skills already.
    struct TickState
    {
        std::vector<std::string> last_running;
        std::vector<BT::NodeStatus> last_statuses;
        std::chrono::steady_clock::time_point last_status;
        bool held{false};
    };

    explicit BtExecutor(const rclcpp::NodeOptions & options);
    void loadTrees();
    void startInterfaces();
    // A tree's declared inputs; nullopt when it declares none.
    [[nodiscard]] std::optional<TreeInputs> inputsOf(const std::string & id) const;
    // The status of a run (or of none when `tree` is null), sent now.
    void publishStatus(const BT::Tree * tree, bool with_xml);
    // The status as one line for monitors, with the running leaves' names.
    void publishMission(const TreeStatus & status, const std::vector<std::string> & running);
    // Halts the running skills (Hold); their parents keep their place.
    static void haltRunningLeaves(BT::Tree & tree);
    rclcpp_action::GoalResponse onGoal(const std::shared_ptr<const RunTree::Goal> & goal);
    rclcpp_action::CancelResponse onCancel();
    void onAccepted(const std::shared_ptr<GoalHandle> & goal);
    void stopForShutdown();
    void run(
        const std::shared_ptr<GoalHandle> & goal,
        const std::vector<std::pair<std::string, std::string>> & entries);
    Outcome tickToEnd(BT::Tree & tree, const std::shared_ptr<GoalHandle> & goal);
    // A paused or held tick: no tick; a Hold halts the running skills once; the status goes out.
    void whilePaused(BT::Tree & tree, TickState & ticks, std::chrono::steady_clock::time_point now);
    // After a tick: the running leaves as feedback and the statuses on change.
    void reportProgress(
        BT::Tree & tree,
        const std::shared_ptr<GoalHandle> & goal,
        TickState & ticks,
        std::chrono::steady_clock::time_point now);
    void reply(
        const std::shared_ptr<GoalHandle> & goal,
        const std::shared_ptr<RunTree::Result> & result,
        Outcome outcome);

    std::vector<std::string> tree_directories_;
    double tick_rate_hz_;
    double status_period_s_;
    int groot2_port_;
    int executor_threads_;
    RosContext context_;
    BT::BehaviorTreeFactory factory_;
    // The tree files and their inputs, loaded once before the interfaces start: read-only after.
    TreeLibrary library_;
    rclcpp_action::Server<RunTree>::SharedPtr server_;
    rclcpp::Publisher<TreeStatus>::SharedPtr status_publisher_;
    // The run in one line for monitors (PlotJuggler, rqt_robot_monitor), with every status.
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr mission_publisher_;
    rclcpp::TimerBase::SharedPtr idle_timer_;
    rclcpp::Service<piper_msgs::srv::ControlTree>::SharedPtr control_service_;
    // Pause and Hold requests for the running tree; the tick loop acts on them.
    std::atomic<bool> pause_{false};
    std::atomic<bool> hold_{false};
    // TreeStatus::state; and what the status reports, guarded by status_mutex_.
    std::atomic<std::uint8_t> state_{TreeStatus::IDLE};
    std::mutex status_mutex_;
    std::string status_tree_;
    // The running tree's last statuses; set and read on the tree's thread only.
    const LastStatus * last_status_{nullptr};
    std::string status_xml_;
    std::string last_result_;
    std::string last_message_;
    // The last run's node statuses and blackboard, sent again while idle.
    std::vector<piper_msgs::msg::TreeNodeStatus> last_nodes_;
    std::vector<piper_msgs::msg::BlackboardEntry> last_blackboard_;
    int status_count_{0};
    // Both sequentially consistent (the default): they are read a few times per tick, so the
    // cost does not matter. stop_: the running tree is to stop. busy_: a run owns the executor.
    std::atomic<bool> stop_{false};
    std::atomic<bool> busy_{false};
    // Set at a shutdown: new goals are refused and a running tree halts.
    std::atomic<bool> shutting_down_{false};
    rclcpp::PreShutdownCallbackHandle shutdown_callback_;
    // Assigned and joined only by onAccepted() and finish(): the action server's callbacks are
    // serialized, onAccepted() runs only once busy_ was false (the thread has ended), and
    // finish() runs after the node stopped spinning or from the destructor.
    std::jthread runner_;
};

}  // namespace piper_bt
