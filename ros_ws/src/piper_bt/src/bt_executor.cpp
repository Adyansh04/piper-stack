#include "piper_bt/bt_executor.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/utils/safe_any.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <format>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "piper_bt/registration.hpp"
#include "piper_bt/ros_leaf.hpp"
#include "piper_bt/tree_library.hpp"

namespace piper_bt
{

namespace
{

std::string joined(const std::vector<std::string> & items)
{
    std::string out;
    for (const std::string & item : items)
    {
        out += out.empty() ? item : std::format(", {}", item);
    }
    return out;
}

std::string blackboardText(const BT::Blackboard & blackboard, const std::string & key)
{
    std::string text;
    return blackboard.get(key, text) ? text : std::string{};
}

// A node status as TreeNodeStatus' code.
std::uint8_t statusCode(BT::NodeStatus status)
{
    using piper_msgs::msg::TreeNodeStatus;
    switch (status)
    {
        case BT::NodeStatus::RUNNING:
            return TreeNodeStatus::RUNNING;
        case BT::NodeStatus::SUCCESS:
            return TreeNodeStatus::SUCCESS;
        case BT::NodeStatus::FAILURE:
            return TreeNodeStatus::FAILURE;
        case BT::NodeStatus::SKIPPED:
            return TreeNodeStatus::SKIPPED;
        case BT::NodeStatus::IDLE:
            break;
    }
    return TreeNodeStatus::IDLE;
}

// A blackboard value as text: strings, numbers, lists of strings and poses; other types by name.
std::string valueText(const BT::Any & value, const std::string & type)
{
    if (value.empty())
    {
        return {};
    }
    if (value.isString())
    {
        return value.cast<std::string>();
    }
    if (value.isNumber())
    {
        return value.isIntegral() ? std::to_string(value.cast<std::int64_t>())
                                  : std::format("{}", value.cast<double>());
    }
    if (value.type() == typeid(std::vector<std::string>))
    {
        return joined(value.cast<std::vector<std::string>>());
    }
    if (value.type() == typeid(geometry_msgs::msg::PoseStamped))
    {
        const auto pose = value.cast<geometry_msgs::msg::PoseStamped>();
        const auto & q = pose.pose.orientation;
        const double yaw = std::atan2(
            2.0 * ((q.w * q.z) + (q.x * q.y)), 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
        return std::format(
            "({:.3f}, {:.3f}, {:.3f}) m, yaw {:.1f} deg in {}",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            yaw * 180.0 / std::numbers::pi,
            pose.header.frame_id);
    }
    return std::format("<{}>", type);
}

// Every subtree's blackboard entries as text, keyed "instance:key" (the root's without a prefix).
std::vector<piper_msgs::msg::BlackboardEntry> blackboardOf(const BT::Tree & tree)
{
    std::vector<piper_msgs::msg::BlackboardEntry> out;
    for (const auto & subtree : tree.subtrees)
    {
        const std::string prefix =
            subtree->instance_name.empty() ? std::string() : subtree->instance_name + ":";
        for (const auto & key : subtree->blackboard->getKeys())
        {
            const auto entry = subtree->blackboard->getEntry(std::string(key));
            if (!entry)
            {
                continue;
            }
            piper_msgs::msg::BlackboardEntry e;
            e.key = prefix + std::string(key);
            e.type = entry->info.typeName();
            {
                const std::scoped_lock entry_lock(entry->entry_mutex);
                e.value = valueText(entry->value, e.type);
            }
            out.push_back(std::move(e));
        }
    }
    return out;
}

double positiveParameter(rclcpp::Node & node, const std::string & name, double value)
{
    const double v = node.declare_parameter<double>(name, value);
    if (!std::isfinite(v) || v <= 0.0)
    {
        throw std::invalid_argument(std::format("{} must be positive, got {}", name, v));
    }
    return v;
}

}  // namespace

// Called on the tree's thread, inside its ticks, as each node changes status.
class BtExecutor::LastStatus : public BT::StatusChangeLogger
{
public:
    explicit LastStatus(const BT::Tree & tree) : BT::StatusChangeLogger(tree.rootNode())
    {
        // A halt is RUNNING to IDLE: the node did not end, it has no last status then.
        enableTransitionToIdle(true);
    }
    ~LastStatus() override
    {
        unsubscribeFromTreeChanges();
    }
    LastStatus(const LastStatus &) = delete;
    LastStatus & operator=(const LastStatus &) = delete;
    LastStatus(LastStatus &&) = delete;
    LastStatus & operator=(LastStatus &&) = delete;

    void callback(
        BT::Duration /*timestamp*/,
        const BT::TreeNode & node,
        BT::NodeStatus prev_status,
        BT::NodeStatus status) override
    {
        if (status != BT::NodeStatus::IDLE)
        {
            last_[node.UID()] = statusCode(status);
        }
        else if (prev_status == BT::NodeStatus::RUNNING)
        {
            last_.erase(node.UID());
        }
    }

    void flush() override {}

    [[nodiscard]] std::uint8_t of(std::uint16_t uid) const
    {
        const auto it = last_.find(uid);
        return it == last_.end() ? piper_msgs::msg::TreeNodeStatus::IDLE : it->second;
    }

private:
    std::map<std::uint16_t, std::uint8_t> last_;
};

std::vector<std::pair<std::string, std::string>> parseBlackboard(
    const std::vector<std::string> & entries)
{
    std::vector<std::pair<std::string, std::string>> out;
    out.reserve(entries.size());
    for (const std::string & entry : entries)
    {
        const auto eq = entry.find('=');
        if (eq == std::string::npos || eq == 0)
        {
            throw std::invalid_argument(
                std::format("blackboard entry '{}' is not key=value", entry));
        }
        out.emplace_back(entry.substr(0, eq), entry.substr(eq + 1));
    }
    return out;
}

std::vector<std::pair<std::string, std::string>> checkBlackboard(
    std::vector<std::pair<std::string, std::string>> entries,
    const std::optional<TreeInputs> & inputs)
{
    if (!inputs)
    {
        return entries;
    }
    std::vector<std::string> names;
    for (const auto & [name, fallback] : *inputs)
    {
        names.push_back(name);
    }
    std::set<std::string> given;
    for (const auto & [key, value] : entries)
    {
        if (!inputs->contains(key))
        {
            throw std::invalid_argument(
                std::format("'{}' is not one of the tree's inputs ({})", key, joined(names)));
        }
        given.insert(key);
    }
    for (const auto & [name, fallback] : *inputs)
    {
        if (given.contains(name))
        {
            continue;
        }
        if (!fallback)
        {
            throw std::invalid_argument(std::format("'{}' is needed ({})", name, joined(names)));
        }
        entries.emplace_back(name, *fallback);
    }
    return entries;
}

std::shared_ptr<BtExecutor> BtExecutor::create(const rclcpp::NodeOptions & options)
{
    // The constructor is private: the leaves' context holds the node, which needs its address.
    std::shared_ptr<BtExecutor> node(new BtExecutor(options));
    node->loadTrees();
    node->startInterfaces();
    return node;
}

BtExecutor::BtExecutor(const rclcpp::NodeOptions & options)
    : rclcpp::Node("bt_executor", options)
    , tree_directories_(
          declare_parameter<std::vector<std::string>>(
              "tree_directories", std::vector<std::string>{"package://piper_bt/trees"}))
    , tick_rate_hz_(positiveParameter(*this, "tick_rate_hz", 20.0))
    , status_period_s_(positiveParameter(*this, "status_period_s", 0.1))
    , groot2_port_(static_cast<int>(declare_parameter<int>("groot2_port", 0)))
    , executor_threads_(static_cast<int>(declare_parameter<int>("executor_threads", 4)))
    , library_(tree_directories_)
{
    // Groot2 binds the port and the one after it.
    if (groot2_port_ < 0 || groot2_port_ > 65534)
    {
        throw std::invalid_argument(
            std::format("groot2_port must be 0 (off) or 1 to 65534, got {}", groot2_port_));
    }
    if (executor_threads_ < 1)
    {
        throw std::invalid_argument(
            std::format("executor_threads must be at least 1, got {}", executor_threads_));
    }
    context_.node = this;
    context_.clients = std::make_shared<Clients>(this);
    context_.stop = &stop_;
    context_.server_timeout_s = positiveParameter(*this, "server_timeout_s", 10.0);
    context_.response_timeout_s = positiveParameter(*this, "response_timeout_s", 30.0);
    context_.goal_timeout_s = positiveParameter(*this, "goal_timeout_s", 900.0);
    context_.short_goal_timeout_s = positiveParameter(*this, "short_goal_timeout_s", 180.0);
    context_.server_lost_s = positiveParameter(*this, "server_lost_s", 10.0);
    context_.halt_wait_s = positiveParameter(*this, "halt_wait_s", 10.0);
    registerLeaves(factory_, context_);
    // A shutdown (docker stop, Ctrl-C) halts the running tree while the context still carries
    // its leaves' cancels.
    shutdown_callback_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
        [this] { stopForShutdown(); });
}

BtExecutor::~BtExecutor()
{
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_callback_);
    finish();
}

void BtExecutor::stopForShutdown()
{
    shutting_down_ = true;
    stop_ = true;
    // The halt cancels the leaves' goals and waits up to halt_wait_s for them to end.
    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::duration<double>(context_.halt_wait_s + 1.0);
    while (busy_ && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void BtExecutor::loadTrees()
{
    library_.load(factory_);
    RCLCPP_INFO(
        get_logger(),
        "Behaviour trees ready: %s",
        joined(factory_.registeredBehaviorTrees()).c_str());
}

void BtExecutor::finish()
{
    stop_ = true;
    if (runner_.joinable())
    {
        runner_.join();
    }
}

std::vector<std::string> BtExecutor::trees() const
{
    return factory_.registeredBehaviorTrees();
}

int BtExecutor::executorThreads() const
{
    return executor_threads_;
}

rclcpp_action::GoalResponse BtExecutor::onGoal(const std::shared_ptr<const RunTree::Goal> & goal)
{
    if (busy_ || shutting_down_)
    {
        RCLCPP_WARN(
            get_logger(),
            "Refused '%s': a tree is running or stopping, or the executor shuts down",
            goal->tree.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    const auto ids = trees();
    if (std::ranges::find(ids, goal->tree) == ids.end())
    {
        RCLCPP_WARN(
            get_logger(),
            "Refused '%s': no such tree (%s)",
            goal->tree.c_str(),
            joined(ids).c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    try
    {
        static_cast<void>(checkBlackboard(parseBlackboard(goal->blackboard), inputsOf(goal->tree)));
    }
    catch (const std::invalid_argument & e)
    {
        RCLCPP_WARN(get_logger(), "Refused '%s': %s", goal->tree.c_str(), e.what());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BtExecutor::onCancel()
{
    // One goal runs at a time: its leaves see this before the tick loop sees the cancel.
    stop_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BtExecutor::onAccepted(const std::shared_ptr<GoalHandle> & goal)
{
    busy_ = true;
    // The previous run's thread has ended: busy_ was false.
    if (runner_.joinable())
    {
        runner_.join();
    }
    // A goal accepted as the shutdown began halts at its first tick.
    stop_ = shutting_down_.load();
    pause_ = false;
    hold_ = false;
    std::vector<std::pair<std::string, std::string>> entries;
    try
    {
        entries = checkBlackboard(
            parseBlackboard(goal->get_goal()->blackboard), inputsOf(goal->get_goal()->tree));
    }
    catch (const std::invalid_argument & e)
    {
        auto result = std::make_shared<RunTree::Result>();
        result->status = "FAILURE";
        result->message = e.what();
        busy_ = false;
        reply(goal, result, Outcome::kFailed);
        return;
    }
    runner_ = std::jthread([this, goal, entries = std::move(entries)] { run(goal, entries); });
}

void BtExecutor::run(
    const std::shared_ptr<GoalHandle> & goal,
    const std::vector<std::pair<std::string, std::string>> & entries)
{
    const std::string id = goal->get_goal()->tree;
    auto blackboard = BT::Blackboard::create();
    for (const auto & [key, value] : entries)
    {
        blackboard->set<std::string>(key, value);
    }
    blackboard->set<bool>("arm_stuck", false);
    Outcome outcome = Outcome::kFailed;
    try
    {
        BT::Tree tree = factory_.createTree(id, blackboard);
        {
            const std::scoped_lock lock(status_mutex_);
            status_tree_ = id;
            status_xml_ = BT::WriteTreeToXML(tree, true, false);
        }
        // Monitoring only: a run goes on without it.
        std::unique_ptr<BT::Groot2Publisher> groot;
        if (groot2_port_ > 0)
        {
            try
            {
                groot = std::make_unique<BT::Groot2Publisher>(tree, groot2_port_);
            }
            catch (const std::exception & e)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Groot2 cannot watch '%s' on port %d: %s",
                    id.c_str(),
                    groot2_port_,
                    e.what());
            }
        }
        RCLCPP_INFO(get_logger(), "Running '%s'", id.c_str());
        const LastStatus last(tree);
        last_status_ = &last;
        state_ = TreeStatus::RUNNING;
        publishStatus(&tree, true);
        outcome = tickToEnd(tree, goal);
        // The last statuses: a client sees where the run ended.
        publishStatus(&tree, false);
        last_status_ = nullptr;
    }
    catch (const std::exception & e)
    {
        // The logger went with the try block.
        last_status_ = nullptr;
        blackboard->set<std::string>(
            "failure", std::format("the tree failed to run: {}", e.what()));
        outcome = Outcome::kFailed;
    }
    auto result = std::make_shared<RunTree::Result>();
    result->success = outcome == Outcome::kSucceeded;
    switch (outcome)
    {
        case Outcome::kSucceeded:
            result->status = "SUCCESS";
            result->message = "done";
            break;
        case Outcome::kFailed:
        {
            result->status = "FAILURE";
            const std::string failure = blackboardText(*blackboard, "failure");
            result->message = failure.empty() ? "the tree failed" : failure;
            break;
        }
        case Outcome::kHalted:
        {
            result->status = "HALTED";
            const std::string halt = blackboardText(*blackboard, "halt_failure");
            result->message = halt.empty() ? "cancelled" : std::format("cancelled, but {}", halt);
            break;
        }
    }
    RCLCPP_INFO(
        get_logger(),
        "'%s' ended %s: %s",
        id.c_str(),
        result->status.c_str(),
        result->message.c_str());
    {
        const std::scoped_lock lock(status_mutex_);
        last_result_ = result->status;
        last_message_ = result->message;
    }
    state_ = TreeStatus::IDLE;
    publishStatus(nullptr, false);
    busy_ = false;
    reply(goal, result, outcome);
}

BtExecutor::Outcome BtExecutor::tickToEnd(BT::Tree & tree, const std::shared_ptr<GoalHandle> & goal)
{
    const auto period = std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::duration<double>(1.0 / tick_rate_hz_));
    TickState ticks;
    ticks.last_status = std::chrono::steady_clock::now();
    while (!goal->is_canceling() && !stop_ && rclcpp::ok())
    {
        const auto now = std::chrono::steady_clock::now();
        if (pause_ || hold_)
        {
            whilePaused(tree, ticks, now);
            tree.sleep(period);
            continue;
        }
        ticks.held = false;
        if (state_.exchange(TreeStatus::RUNNING) != TreeStatus::RUNNING)
        {
            publishStatus(&tree, false);
        }
        const BT::NodeStatus status = tree.tickOnce();
        if (status != BT::NodeStatus::RUNNING)
        {
            return status == BT::NodeStatus::SUCCESS ? Outcome::kSucceeded : Outcome::kFailed;
        }
        reportProgress(tree, goal, ticks, now);
        tree.sleep(period);
    }
    // Leaves that would start something fail at once while the halt runs; the running ones cancel
    // their goals and wait for them to stop.
    stop_ = true;
    tree.haltTree();
    return Outcome::kHalted;
}

void BtExecutor::whilePaused(
    BT::Tree & tree, TickState & ticks, std::chrono::steady_clock::time_point now)
{
    // Held: the running skills halt once; Resume starts them again.
    if (hold_ && !ticks.held)
    {
        haltRunningLeaves(tree);
        ticks.held = true;
    }
    const std::uint8_t state = hold_ ? TreeStatus::HELD : TreeStatus::PAUSED;
    if (state_.exchange(state) != state || now - ticks.last_status >= std::chrono::seconds(1))
    {
        publishStatus(&tree, false);
        ticks.last_status = now;
    }
}

void BtExecutor::reportProgress(
    BT::Tree & tree,
    const std::shared_ptr<GoalHandle> & goal,
    TickState & ticks,
    std::chrono::steady_clock::time_point now)
{
    std::vector<std::string> running;
    tree.applyVisitor([&running](BT::TreeNode * node) {
        if (node->status() == BT::NodeStatus::RUNNING && node->type() == BT::NodeType::ACTION)
        {
            running.push_back(node->fullPath());
        }
    });
    if (running != ticks.last_running)
    {
        auto feedback = std::make_shared<RunTree::Feedback>();
        feedback->running = running;
        goal->publish_feedback(feedback);
        ticks.last_running = std::move(running);
    }
    // Statuses on change, at most every status_period_s, and once a second.
    std::vector<BT::NodeStatus> statuses;
    tree.applyVisitor(
        [&statuses](const BT::TreeNode * node) { statuses.push_back(node->status()); });
    const auto since = now - ticks.last_status;
    if ((statuses != ticks.last_statuses
         && since >= std::chrono::duration<double>(status_period_s_))
        || since >= std::chrono::seconds(1))
    {
        publishStatus(&tree, false);
        ticks.last_status = now;
        ticks.last_statuses = std::move(statuses);
    }
}

void BtExecutor::reply(
    const std::shared_ptr<GoalHandle> & goal,
    const std::shared_ptr<RunTree::Result> & result,
    Outcome outcome)
{
    try
    {
        if (outcome == Outcome::kHalted && goal->is_canceling())
        {
            goal->canceled(result);
        }
        else if (outcome == Outcome::kSucceeded)
        {
            goal->succeed(result);
        }
        else
        {
            goal->abort(result);
        }
    }
    catch (const std::exception & e)
    {
        // After shutdown the goal can no longer be answered.
        RCLCPP_WARN(
            get_logger(),
            "The result of '%s' was not sent: %s",
            goal->get_goal()->tree.c_str(),
            e.what());
    }
}

std::optional<TreeInputs> BtExecutor::inputsOf(const std::string & id) const
{
    const auto it = library_.inputs().find(id);
    return it == library_.inputs().end() ? std::nullopt : std::optional<TreeInputs>(it->second);
}

void BtExecutor::startInterfaces()
{
    using piper_msgs::srv::ControlTree;
    server_ = rclcpp_action::create_server<RunTree>(
        this,
        "run_tree",
        [this](const rclcpp_action::GoalUUID &, const std::shared_ptr<const RunTree::Goal> & goal) {
            return onGoal(goal);
        },
        [this](const std::shared_ptr<GoalHandle> &) { return onCancel(); },
        [this](const std::shared_ptr<GoalHandle> & goal) { onAccepted(goal); });
    // The last status stays for a client that connects later.
    status_publisher_ =
        create_publisher<TreeStatus>("~/status", rclcpp::QoS(1).reliable().transient_local());
    mission_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(10));
    idle_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
        if (!busy_)
        {
            publishStatus(nullptr, false);
        }
    });
    control_service_ = create_service<ControlTree>(
        "~/control",
        [this](
            const std::shared_ptr<ControlTree::Request> & req,
            const std::shared_ptr<ControlTree::Response> & res) {
            if (!busy_)
            {
                res->message = "no tree is running";
                return;
            }
            switch (req->command)
            {
                case ControlTree::Request::PAUSE:
                    pause_ = true;
                    res->message = "paused: the running skills finish their step";
                    break;
                case ControlTree::Request::HOLD:
                    hold_ = true;
                    res->message = "held: the running skills halt";
                    break;
                case ControlTree::Request::RESUME:
                    pause_ = false;
                    hold_ = false;
                    res->message = "resumed";
                    break;
                default:
                    res->message = std::format(
                        "command {} is not PAUSE (1), HOLD (2) or RESUME (3)", req->command);
                    return;
            }
            res->success = true;
            RCLCPP_INFO(get_logger(), "Control: %s", res->message.c_str());
        });
}

void BtExecutor::publishStatus(const BT::Tree * tree, bool with_xml)
{
    TreeStatus msg;
    msg.header.stamp = now();
    msg.state = state_;
    {
        const std::scoped_lock lock(status_mutex_);
        msg.tree = status_tree_;
        msg.result = last_result_;
        msg.message = last_message_;
        // The structure now and then too, for a client that missed the run's start.
        if (tree != nullptr && (with_xml || ++status_count_ % 10 == 0))
        {
            msg.xml = status_xml_;
        }
    }
    std::vector<std::string> running;
    if (tree != nullptr)
    {
        tree->applyVisitor([this, &msg, &running](const BT::TreeNode * node) {
            piper_msgs::msg::TreeNodeStatus s;
            s.uid = node->UID();
            s.status = statusCode(node->status());
            s.last = last_status_ != nullptr ? last_status_->of(node->UID()) : s.status;
            msg.nodes.push_back(s);
            if (node->status() == BT::NodeStatus::RUNNING && node->type() == BT::NodeType::ACTION)
            {
                running.push_back(node->name().empty() ? node->registrationName() : node->name());
            }
        });
        msg.blackboard = blackboardOf(*tree);
        const std::scoped_lock lock(status_mutex_);
        last_nodes_ = msg.nodes;
        last_blackboard_ = msg.blackboard;
    }
    else
    {
        // Idle: the last run as it ended, so a client sees that even when a later status replaced
        // the run's final one before it arrived.
        const std::scoped_lock lock(status_mutex_);
        msg.nodes = last_nodes_;
        msg.blackboard = last_blackboard_;
        if (!msg.nodes.empty() && ++status_count_ % 10 == 0)
        {
            msg.xml = status_xml_;
        }
    }
    status_publisher_->publish(msg);
    publishMission(msg, running);
}

void BtExecutor::publishMission(const TreeStatus & status, const std::vector<std::string> & running)
{
    using diagnostic_msgs::msg::DiagnosticStatus;
    DiagnosticStatus d;
    d.name = std::format("{}: mission", get_name());
    const auto value = [&d](const std::string & key, const std::string & text) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = text;
        d.values.push_back(std::move(kv));
    };
    switch (status.state)
    {
        case TreeStatus::RUNNING:
            d.level = DiagnosticStatus::OK;
            d.message = running.empty()
                            ? std::format("{} running", status.tree)
                            : std::format("{} running: {}", status.tree, joined(running));
            break;
        case TreeStatus::PAUSED:
            d.level = DiagnosticStatus::WARN;
            d.message = std::format("{} paused", status.tree);
            break;
        case TreeStatus::HELD:
            d.level = DiagnosticStatus::WARN;
            d.message = std::format("{} held", status.tree);
            break;
        default:
            d.level = DiagnosticStatus::OK;
            if (status.result == "FAILURE")
            {
                d.level = DiagnosticStatus::ERROR;
            }
            else if (status.result == "HALTED")
            {
                d.level = DiagnosticStatus::WARN;
            }
            d.message =
                status.result.empty()
                    ? std::string("idle")
                    : std::format(
                          "idle; {} ended {}: {}", status.tree, status.result, status.message);
            break;
    }
    value("tree", status.tree);
    value("running", joined(running));
    value("result", status.result);
    value("message", status.message);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = status.header.stamp;
    array.status.push_back(std::move(d));
    mission_publisher_->publish(array);
}

void BtExecutor::haltRunningLeaves(BT::Tree & tree)
{
    std::vector<BT::TreeNode *> running;
    tree.applyVisitor([&running](BT::TreeNode * node) {
        if (node->status() == BT::NodeStatus::RUNNING && node->type() == BT::NodeType::ACTION)
        {
            running.push_back(node);
        }
    });
    // Each leaf cancels its skill's goal and waits for it to stop; its parent keeps its place.
    for (BT::TreeNode * node : running)
    {
        node->haltNode();
    }
}

}  // namespace piper_bt
