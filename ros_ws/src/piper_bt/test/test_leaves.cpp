// The leaves' logic, the action and service leaves against in-process servers, and the executor's
// run_tree.
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <gtest/gtest.h>
#include <behaviortree_cpp/utils/safe_any.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <typeindex>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <piper_msgs/action/move_arm_to_named_pose.hpp>
#include <piper_msgs/action/run_tree.hpp>
#include <piper_msgs/msg/tree_node_status.hpp>
#include <piper_msgs/msg/tree_status.hpp>
#include <piper_msgs/srv/control_tree.hpp>
#include <piper_msgs/srv/place_target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "piper_bt/bt_executor.hpp"
#include "piper_bt/port_types.hpp"
#include "piper_bt/registration.hpp"
#include "piper_bt/ros_leaf.hpp"

namespace
{

using MoveArm = piper_msgs::action::MoveArmToNamedPose;
using RunTree = piper_msgs::action::RunTree;
using PlaceTargetSrv = piper_msgs::srv::PlaceTarget;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

// The arm skill server: "ok" succeeds after 0.2 s, "fail" aborts with a reason, "refuse" is
// rejected, "slow" runs until cancelled, "stubborn" runs 3 s whatever is asked.
class FakeArm
{
public:
    explicit FakeArm(const rclcpp::Node::SharedPtr & node)
    {
        server_ = rclcpp_action::create_server<MoveArm>(
            node,
            "move_arm_to_named_pose",
            [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveArm::Goal> & goal) {
                return goal->name == "refuse" ? rclcpp_action::GoalResponse::REJECT
                                              : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> &) {
                cancelled_ = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> & goal) {
                ++goals_;
                workers_.emplace_back([goal] { execute(goal); });
            });
    }

    [[nodiscard]] bool cancelled() const
    {
        return cancelled_;
    }

    // How many goals it took.
    [[nodiscard]] int goals() const
    {
        return goals_;
    }

private:
    static void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveArm>> & goal)
    {
        auto result = std::make_shared<MoveArm::Result>();
        const std::string name = goal->get_goal()->name;
        const bool stubborn = name == "stubborn";
        std::chrono::milliseconds runs = 200ms;
        if (name == "slow")
        {
            runs = 30s;
        }
        else if (stubborn)
        {
            runs = 3s;
        }
        const auto end = std::chrono::steady_clock::now() + runs;
        while (std::chrono::steady_clock::now() < end && rclcpp::ok())
        {
            if (goal->is_canceling() && !stubborn)
            {
                goal->canceled(result);
                return;
            }
            std::this_thread::sleep_for(10ms);
        }
        result->success = name != "fail";
        result->message = result->success ? "at " + name : "no named pose 'fail'";
        if (result->success)
        {
            goal->succeed(result);
        }
        else
        {
            goal->abort(result);
        }
    }

    std::atomic<bool> cancelled_{false};
    std::atomic<int> goals_{0};
    rclcpp_action::Server<MoveArm>::SharedPtr server_;
    std::vector<std::jthread> workers_;
};

// The place_target service: answers after `delay`, and keeps the last request's anchor pose.
class FakePlaceTarget
{
public:
    FakePlaceTarget(const rclcpp::Node::SharedPtr & node, std::chrono::milliseconds delay)
    {
        service_ = node->create_service<PlaceTargetSrv>(
            "place_target",
            [delay, asked = asked_](
                const std::shared_ptr<PlaceTargetSrv::Request> & request,
                const std::shared_ptr<PlaceTargetSrv::Response> & response) {
                std::this_thread::sleep_for(delay);
                {
                    const std::scoped_lock lock(asked->mutex);
                    asked->anchor_pose = request->anchor_pose;
                }
                response->success = true;
                response->target.header.frame_id = "world";
            });
    }

    [[nodiscard]] geometry_msgs::msg::PoseStamped anchorPose() const
    {
        const std::scoped_lock lock(asked_->mutex);
        return asked_->anchor_pose;
    }

private:
    struct Asked
    {
        std::mutex mutex;
        // Guarded by mutex: the service's thread writes it.
        geometry_msgs::msg::PoseStamped anchor_pose;
    };
    std::shared_ptr<Asked> asked_ = std::make_shared<Asked>();
    rclcpp::Service<PlaceTargetSrv>::SharedPtr service_;
};

// A node spun on its own thread, as the executor's main does.
class Spun
{
public:
    explicit Spun(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
        executor_.add_node(node_);
        thread_ = std::jthread([this] { executor_.spin(); });
    }
    ~Spun()
    {
        executor_.cancel();
    }
    Spun(const Spun &) = delete;
    Spun & operator=(const Spun &) = delete;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 2};
    std::jthread thread_;
};

// A directory of its own under the system's temp directory, removed at the end.
class TempDir
{
public:
    TempDir()
    {
        std::random_device random;
        // A name taken already makes create_directory false: try another.
        while (true)
        {
            path_ = std::filesystem::temp_directory_path()
                    / std::format("piper_bt_{:08x}{:08x}", random(), random());
            if (std::filesystem::create_directory(path_))
            {
                break;
            }
        }
    }
    ~TempDir()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    TempDir(const TempDir &) = delete;
    TempDir & operator=(const TempDir &) = delete;

    [[nodiscard]] const std::filesystem::path & path() const
    {
        return path_;
    }

private:
    std::filesystem::path path_;
};

// Leaves on a node of their own, with short limits.
struct Leaves
{
    explicit Leaves(const std::string & name) : node(std::make_shared<rclcpp::Node>(name))
    {
        context.node = node.get();
        context.clients = std::make_shared<piper_bt::Clients>(node.get());
        context.halt_wait_s = 2.0;
        context.server_timeout_s = 0.3;
        piper_bt::registerLeaves(factory, context);
    }

    rclcpp::Node::SharedPtr node;
    piper_bt::RosContext context;
    BT::BehaviorTreeFactory factory;
};

BT::NodeStatus tickUntilDone(BT::Tree & tree, std::chrono::milliseconds limit)
{
    const auto end = std::chrono::steady_clock::now() + limit;
    BT::NodeStatus status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end)
    {
        std::this_thread::sleep_for(10ms);
        status = tree.tickOnce();
    }
    return status;
}

std::string treeWithArm(const std::string & pose, const std::string & extra = "")
{
    return R"(<root BTCPP_format="4"><BehaviorTree ID="T"><MoveArmToNamedPose pose=")" + pose
           + R"(" server_timeout_s="5" )" + extra + "/></BehaviorTree></root>";
}

std::string rootText(BT::Tree & tree, const std::string & key)
{
    std::string text;
    return tree.rootBlackboard()->get(key, text) ? text : std::string{};
}

}  // namespace

TEST(Blackboard, ParsesKeyValueEntries)
{
    const auto pairs = piper_bt::parseBlackboard({"object=red cube", "region=", "a=b=c"});
    ASSERT_EQ(pairs.size(), 3U);
    EXPECT_EQ(pairs[0].first, "object");
    EXPECT_EQ(pairs[0].second, "red cube");
    EXPECT_EQ(pairs[1].second, "");
    EXPECT_EQ(pairs[2].second, "b=c");
    EXPECT_THROW(
        static_cast<void>(piper_bt::parseBlackboard({"no equals"})), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(piper_bt::parseBlackboard({"=value"})), std::invalid_argument);
}

TEST(Ports, PoseFromText)
{
    const auto pose = BT::convertFromString<geometry_msgs::msg::PoseStamped>("1.5;-2;3.14159265");
    EXPECT_EQ(pose.header.frame_id, "");  // NavigateToPose fills in the map frame
    EXPECT_NEAR(pose.pose.position.x, 1.5, 1e-9);
    EXPECT_NEAR(pose.pose.position.y, -2.0, 1e-9);
    EXPECT_NEAR(pose.pose.orientation.z, 1.0, 1e-6);  // half a turn
    EXPECT_EQ(
        BT::convertFromString<geometry_msgs::msg::PoseStamped>("0; 0 ;0;odom").header.frame_id,
        "odom");
}

class BadPose : public ::testing::TestWithParam<const char *>
{};

TEST_P(BadPose, IsRefused)
{
    EXPECT_THROW(
        static_cast<void>(BT::convertFromString<geometry_msgs::msg::PoseStamped>(GetParam())),
        BT::RuntimeError)
        << GetParam();
}

INSTANTIATE_TEST_SUITE_P(
    Texts,
    BadPose,
    ::testing::Values("1;2", "1,5;2;0", "1.5m;2;0", "nan;0;0", "0;inf;0", "1;2;3;4;5", "1;2;0; "));

TEST(Ports, EveryPoseStampedPortParsesText)
{
    const Leaves leaves("test_pose_ports");
    int checked = 0;
    for (const auto & [id, manifest] : leaves.factory.manifests())
    {
        for (const auto & [port, info] : manifest.ports)
        {
            if (info.type() != std::type_index(typeid(geometry_msgs::msg::PoseStamped))
                || info.direction() != BT::PortDirection::INPUT)
            {
                continue;
            }
            const BT::Any parsed = info.converter()("1;2;0");
            EXPECT_NEAR(parsed.cast<geometry_msgs::msg::PoseStamped>().pose.position.y, 2.0, 1e-12)
                << id << "." << port;
            ++checked;
        }
    }
    EXPECT_GE(checked, 1);
}

TEST(ActionLeaf, SucceedsFailsWithTheReasonAndStopsOnHalt)
{
    Leaves leaves("test_action_leaf");
    const FakeArm arm(leaves.node);
    const Spun spun(leaves.node);

    auto ok = leaves.factory.createTreeFromText(treeWithArm("ok"));
    EXPECT_EQ(tickUntilDone(ok, 10s), BT::NodeStatus::SUCCESS);

    auto fail = leaves.factory.createTreeFromText(treeWithArm("fail"));
    EXPECT_EQ(tickUntilDone(fail, 10s), BT::NodeStatus::FAILURE);
    EXPECT_NE(rootText(fail, "failure").find("no named pose 'fail'"), std::string::npos);

    auto refused = leaves.factory.createTreeFromText(treeWithArm("refuse"));
    EXPECT_EQ(tickUntilDone(refused, 10s), BT::NodeStatus::FAILURE);
    EXPECT_NE(rootText(refused, "failure").find("refused"), std::string::npos);

    auto slow = leaves.factory.createTreeFromText(treeWithArm("slow"));
    EXPECT_EQ(tickUntilDone(slow, 1s), BT::NodeStatus::RUNNING);
    // The halt waits for the server's answer to the cancel.
    slow.haltTree();
    EXPECT_TRUE(arm.cancelled());
    EXPECT_EQ(rootText(slow, "halt_failure"), "");
}

TEST(ActionLeaf, AGoalThatRunsTooLongIsCancelled)
{
    Leaves leaves("test_action_timeout");
    const FakeArm arm(leaves.node);
    const Spun spun(leaves.node);
    auto slow = leaves.factory.createTreeFromText(treeWithArm("slow", R"(timeout_s="0.5")"));
    EXPECT_EQ(tickUntilDone(slow, 10s), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(arm.cancelled());
    EXPECT_NE(rootText(slow, "failure").find("did not finish within"), std::string::npos);
}

TEST(ActionLeaf, AGoalThatIgnoresTheCancelIsNamed)
{
    Leaves leaves("test_action_stubborn");
    const FakeArm arm(leaves.node);
    const Spun spun(leaves.node);
    auto stubborn = leaves.factory.createTreeFromText(treeWithArm("stubborn"));
    EXPECT_EQ(tickUntilDone(stubborn, 500ms), BT::NodeStatus::RUNNING);
    stubborn.haltTree();
    EXPECT_NE(rootText(stubborn, "halt_failure").find("may still be running"), std::string::npos);
}

TEST(ActionLeaf, NoServerFailsWithoutBlockingATick)
{
    Leaves leaves("test_action_no_server");
    const Spun spun(leaves.node);
    auto tree = leaves.factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="T"><MoveArmToNamedPose pose="ok"/></BehaviorTree></root>)");
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
    EXPECT_EQ(tickUntilDone(tree, 5s), BT::NodeStatus::FAILURE);
    EXPECT_NE(rootText(tree, "failure").find("no action server"), std::string::npos);
}

TEST(ServiceLeaf, ASlowResponseFails)
{
    Leaves leaves("test_service_slow");
    auto server = std::make_shared<rclcpp::Node>("test_slow_place_target");
    const FakePlaceTarget place_target(server, 3000ms);
    const Spun spun_server(server);
    const Spun spun(leaves.node);
    leaves.context.server_timeout_s = 5.0;
    BT::BehaviorTreeFactory factory;
    piper_bt::registerLeaves(factory, leaves.context);
    auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">
        <PlaceTarget mode="metric" x="0.0" y="0.0" response_timeout_s="0.5"/>
      </BehaviorTree></root>)");
    EXPECT_EQ(tickUntilDone(tree, 10s), BT::NodeStatus::FAILURE);
    const auto failure = rootText(tree, "failure");
    EXPECT_NE(failure.find("did not answer"), std::string::npos) << failure;
}

TEST(ServiceLeaf, AnOptionalPoseIsNoneByDefaultOrTextOrAnEntry)
{
    Leaves leaves("test_optional_pose");
    auto server = std::make_shared<rclcpp::Node>("test_anchor_place_target");
    const FakePlaceTarget place_target(server, 0ms);
    const Spun spun_server(server);
    const Spun spun(leaves.node);
    // One PlaceTarget with these attributes, on a blackboard filled first, as a run's is.
    const auto run = [&](const std::string & attributes,
                         const std::function<void(BT::Blackboard &)> & fill) {
        auto blackboard = BT::Blackboard::create();
        fill(*blackboard);
        auto tree = leaves.factory.createTreeFromText(
            std::format(
                R"(<root BTCPP_format="4"><BehaviorTree ID="T"><PlaceTarget mode="anchor" {}/></BehaviorTree></root>)",
                attributes),
            blackboard);
        const auto status = tickUntilDone(tree, 5s);
        return std::pair{status, rootText(tree, "failure")};
    };
    const auto nothing = [](BT::Blackboard &) {
    };

    EXPECT_EQ(run("", nothing).first, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(place_target.anchorPose().header.frame_id, "");
    EXPECT_EQ(
        run(R"(anchor_pose="{a}")", [](BT::Blackboard & b) { b.set<std::string>("a", "none"); })
            .first,
        BT::NodeStatus::SUCCESS);
    EXPECT_EQ(place_target.anchorPose().header.frame_id, "");

    geometry_msgs::msg::PoseStamped placed;
    placed.header.frame_id = "world";
    placed.pose.position.x = 0.3;
    EXPECT_EQ(
        run(R"(anchor_pose="{a}")", [&placed](BT::Blackboard & b) { b.set("a", placed); }).first,
        BT::NodeStatus::SUCCESS);
    EXPECT_EQ(place_target.anchorPose().header.frame_id, "world");
    EXPECT_NEAR(place_target.anchorPose().pose.position.x, 0.3, 1e-12);

    EXPECT_EQ(run(R"(anchor_pose="0.2;0.1;0;world")", nothing).first, BT::NodeStatus::SUCCESS);
    EXPECT_NEAR(place_target.anchorPose().pose.position.y, 0.1, 1e-12);

    // Text that is no pose: a literal does not build, an entry's fails the leaf.
    EXPECT_THROW(
        static_cast<void>(run(R"(anchor_pose="beside it")", nothing)), BT::BehaviorTreeException);
    const auto [status, failure] = run(
        R"(anchor_pose="{a}")", [](BT::Blackboard & b) { b.set<std::string>("a", "beside it"); });
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_NE(failure.find("a pose is"), std::string::npos) << failure;
}

TEST(Executor, RunsTreesRefusesBadGoalsAndCancels)
{
    const TempDir dir;
    std::ofstream(dir.path() / "trees.xml") << R"(<root BTCPP_format="4">
        <BehaviorTree ID="Fine"><AlwaysSuccess/></BehaviorTree>
        <BehaviorTree ID="Arm">
          <MoveArmToNamedPose pose="{pose}" server_timeout_s="5"/>
        </BehaviorTree>
        <TreeNodesModel>
          <SubTree ID="Arm"><input_port name="pose"/></SubTree>
        </TreeNodesModel>
      </root>)";
    rclcpp::NodeOptions options;
    options.append_parameter_override(
        "tree_directories", std::vector<std::string>{dir.path().string()});
    // A taken Groot2 port costs only the monitoring.
    BT::BehaviorTreeFactory blocker_factory;
    const auto blocker_tree = blocker_factory.createTreeFromText(
        R"(<root BTCPP_format="4"><BehaviorTree ID="B"><AlwaysSuccess/></BehaviorTree></root>)");
    const BT::Groot2Publisher blocker(blocker_tree, 17667);
    options.append_parameter_override("groot2_port", 17667);
    auto executor = piper_bt::BtExecutor::create(options);
    auto client_node = std::make_shared<rclcpp::Node>("test_run_tree");
    const FakeArm arm(client_node);
    auto client = rclcpp_action::create_client<RunTree>(client_node, "run_tree");
    {
        const Spun spun_executor(executor);
        const Spun spun_client(client_node);
        ASSERT_TRUE(client->wait_for_action_server(5s));

        // The handle, or null when the executor refuses the goal.
        const auto send = [&](const std::string & tree,
                              const std::vector<std::string> & blackboard) {
            RunTree::Goal goal;
            goal.tree = tree;
            goal.blackboard = blackboard;
            return client->async_send_goal(goal).get();
        };
        const auto result = [&](const auto & handle) {
            return *client->async_get_result(handle).get().result;
        };
        const auto fine = send("Fine", {});
        ASSERT_TRUE(fine);
        EXPECT_EQ(result(fine).status, "SUCCESS");
        const auto failed = send("Arm", {"pose=fail"});
        ASSERT_TRUE(failed);
        EXPECT_NE(result(failed).message.find("no named pose 'fail'"), std::string::npos);
        EXPECT_FALSE(send("NoSuchTree", {}));
        EXPECT_FALSE(send("Fine", {"bad entry"}));
        EXPECT_FALSE(send("Arm", {"pose=ok", "speed=1"}));  // not an input of Arm
        EXPECT_FALSE(send("Arm", {}));                      // pose is needed

        const auto slow = send("Arm", {"pose=slow"});
        ASSERT_TRUE(slow);
        std::this_thread::sleep_for(500ms);
        EXPECT_FALSE(send("Fine", {}));  // busy
        client->async_cancel_goal(slow);
        const auto halted = result(slow);
        EXPECT_EQ(halted.status, "HALTED");
        EXPECT_EQ(halted.message, "cancelled");
        EXPECT_TRUE(arm.cancelled());
    }
    executor->finish();
}

TEST(Executor, AShutdownHaltsTheRunningTreeWhileItsCancelsGetOut)
{
    const TempDir dir;
    std::ofstream(dir.path() / "trees.xml") << R"(<root BTCPP_format="4">
        <BehaviorTree ID="Arm"><MoveArmToNamedPose pose="slow" server_timeout_s="5"/></BehaviorTree>
      </root>)";
    // The executor on a context of its own, which the test shuts down as a signal would.
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    options.append_parameter_override(
        "tree_directories", std::vector<std::string>{dir.path().string()});
    options.append_parameter_override("groot2_port", 0);
    auto executor = piper_bt::BtExecutor::create(options);
    rclcpp::ExecutorOptions spin_options;
    spin_options.context = context;
    rclcpp::executors::MultiThreadedExecutor spinner(spin_options, 2);
    spinner.add_node(executor);
    std::jthread spin([&spinner] { spinner.spin(); });

    auto client_node = std::make_shared<rclcpp::Node>("test_shutdown");
    const FakeArm arm(client_node);
    auto client = rclcpp_action::create_client<RunTree>(client_node, "run_tree");
    const Spun spun_client(client_node);
    ASSERT_TRUE(client->wait_for_action_server(5s));
    RunTree::Goal goal;
    goal.tree = "Arm";
    ASSERT_TRUE(client->async_send_goal(goal).get());
    std::this_thread::sleep_for(1s);
    context->shutdown("test");
    EXPECT_TRUE(arm.cancelled());
    spin.join();
    executor->finish();
}

TEST(Executor, PauseLetsTheStepEndHoldHaltsItAndResumeGoesOn)
{
    using ControlTree = piper_msgs::srv::ControlTree;
    using TreeStatus = piper_msgs::msg::TreeStatus;
    const TempDir dir;
    std::ofstream(dir.path() / "trees.xml") << R"(<root BTCPP_format="4">
        <BehaviorTree ID="TwoMoves">
          <Sequence>
            <MoveArmToNamedPose pose="ok" server_timeout_s="5"/>
            <MoveArmToNamedPose pose="ok" server_timeout_s="5"/>
          </Sequence>
        </BehaviorTree>
        <BehaviorTree ID="Slow"><MoveArmToNamedPose pose="slow" server_timeout_s="5"/></BehaviorTree>
      </root>)";
    rclcpp::NodeOptions options;
    options.append_parameter_override(
        "tree_directories", std::vector<std::string>{dir.path().string()});
    options.append_parameter_override("groot2_port", 0);
    auto executor = piper_bt::BtExecutor::create(options);
    auto client_node = std::make_shared<rclcpp::Node>("test_control");
    const FakeArm arm(client_node);
    auto client = rclcpp_action::create_client<RunTree>(client_node, "run_tree");
    auto control = client_node->create_client<ControlTree>("bt_executor/control");
    std::atomic<std::uint8_t> state{TreeStatus::IDLE};
    // The most nodes one status showed as having succeeded in this run, and whether the last
    // status of a halted run still showed a node running.
    std::atomic<int> succeeded{0};
    std::atomic<bool> stale_running{false};
    const auto status = client_node->create_subscription<TreeStatus>(
        "bt_executor/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [&state, &succeeded, &stale_running](const TreeStatus & msg) {
            state = msg.state;
            const auto n = static_cast<int>(std::ranges::count_if(msg.nodes, [](const auto & node) {
                return node.last == piper_msgs::msg::TreeNodeStatus::SUCCESS;
            }));
            succeeded = std::max(succeeded.load(), n);
            if (msg.result == "HALTED" || msg.state == TreeStatus::HELD)
            {
                stale_running = std::ranges::any_of(msg.nodes, [](const auto & node) {
                    return node.status != piper_msgs::msg::TreeNodeStatus::RUNNING
                           && node.last == piper_msgs::msg::TreeNodeStatus::RUNNING;
                });
            }
        });
    {
        const Spun spun_executor(executor);
        const Spun spun_client(client_node);
        ASSERT_TRUE(client->wait_for_action_server(5s));
        ASSERT_TRUE(control->wait_for_service(5s));
        const auto command = [&](std::uint8_t c) {
            auto request = std::make_shared<ControlTree::Request>();
            request->command = c;
            return control->async_send_request(request).get()->success;
        };
        const auto within = [](const std::function<bool()> & done) {
            const auto end = std::chrono::steady_clock::now() + 5s;
            while (!done() && std::chrono::steady_clock::now() < end)
            {
                std::this_thread::sleep_for(10ms);
            }
            return done();
        };
        EXPECT_FALSE(command(ControlTree::Request::PAUSE));  // nothing runs

        RunTree::Goal goal;
        goal.tree = "TwoMoves";
        const auto paused = client->async_send_goal(goal).get();
        ASSERT_TRUE(paused);
        ASSERT_TRUE(within([&] { return arm.goals() == 1; }));
        ASSERT_TRUE(command(ControlTree::Request::PAUSE));
        EXPECT_TRUE(within([&] { return state == TreeStatus::PAUSED; }));
        // The first move ends on its own (0.2 s); the second does not start while paused.
        auto first = client->async_get_result(paused);
        EXPECT_EQ(first.wait_for(1s), std::future_status::timeout);
        EXPECT_EQ(arm.goals(), 1);
        ASSERT_TRUE(command(ControlTree::Request::RESUME));
        EXPECT_EQ(first.get().result->status, "SUCCESS");
        EXPECT_EQ(arm.goals(), 2);
        // The Sequence and both moves, though the finished tree reset them to IDLE.
        EXPECT_TRUE(within([&] { return succeeded >= 3; })) << succeeded;

        goal.tree = "Slow";
        const auto held = client->async_send_goal(goal).get();
        ASSERT_TRUE(held);
        ASSERT_TRUE(within([&] { return arm.goals() == 3; }));
        ASSERT_TRUE(command(ControlTree::Request::HOLD));
        EXPECT_TRUE(within([&] { return state == TreeStatus::HELD; }));
        EXPECT_TRUE(arm.cancelled());
        // Resume runs the halted step again; Cancel ends the run.
        ASSERT_TRUE(command(ControlTree::Request::RESUME));
        EXPECT_TRUE(within([&] { return arm.goals() == 4; }));
        client->async_cancel_goal(held);
        EXPECT_EQ(client->async_get_result(held).get().result->status, "HALTED");
        EXPECT_TRUE(within([&] { return state == TreeStatus::IDLE; }));
        // A halted node has no last status: nothing shows as running once the run stopped.
        EXPECT_FALSE(stale_running);
    }
    executor->finish();
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int failed = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return failed;
}
