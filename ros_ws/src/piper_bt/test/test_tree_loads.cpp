// The shipped trees build against the registered leaves and behave as their comments say, with
// scripted stand-ins for the leaves and no ROS graph.
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "piper_bt/bt_executor.hpp"
#include "piper_bt/registration.hpp"
#include "piper_bt/ros_leaf.hpp"
#include "piper_bt/skills/find_objects.hpp"
#include "piper_bt/skills/move_arm_to_named_pose.hpp"
#include "piper_bt/skills/pick_object.hpp"
#include "piper_bt/skills/place_object.hpp"
#include "piper_bt/skills/place_target.hpp"
#include "piper_bt/skills/run_stage.hpp"
#include "piper_bt/skills/view_workspace.hpp"
#include "piper_bt/tree_library.hpp"

namespace
{

class TreeLoads : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }
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

    void write(const std::string & file, const std::string & text) const
    {
        std::ofstream(path_ / file) << text;
    }

private:
    std::filesystem::path path_;
};

std::shared_ptr<piper_bt::BtExecutor> executorOn(const std::string & dir)
{
    rclcpp::NodeOptions options;
    options.append_parameter_override("tree_directories", std::vector<std::string>{dir});
    options.append_parameter_override("groot2_port", 0);
    return piper_bt::BtExecutor::create(options);
}

std::string readFile(const std::filesystem::path & path)
{
    std::ifstream in(path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

using Entries = std::vector<std::pair<std::string, std::string>>;

// What a scripted leaf does: its status for a tick, by its name in the tree and its ports.
using Script = std::function<BT::NodeStatus(const std::string & name, BT::TreeNode & leaf)>;

// A stand-in for a real leaf: the same ports, the outcome from the script, every tick recorded.
template <typename Real>
class Scripted : public BT::SyncActionNode
{
public:
    Scripted(
        const std::string & name,
        const BT::NodeConfig & config,
        Script script,
        std::vector<std::string> & log)
        : BT::SyncActionNode(name, config), script_(std::move(script)), log_(log)
    {}

    static BT::PortsList providedPorts()
    {
        return Real::providedPorts();
    }

    BT::NodeStatus tick() override
    {
        log_.push_back(name());
        // The real leaf reads every input: one that points at a missing entry would fail it.
        for (const auto & [port, remapped] : config().input_ports)
        {
            BT::StringView key;
            if (BT::TreeNode::isBlackboardPointer(remapped, &key)
                && !config().blackboard->getEntry(std::string(key)))
            {
                ADD_FAILURE() << name() << "." << port << " reads " << remapped
                              << ", which nothing set";
            }
        }
        return script_(name(), *this);
    }

private:
    Script script_;
    std::vector<std::string> & log_;
};

template <typename Real>
void registerScripted(
    BT::BehaviorTreeFactory & factory,
    const std::string & id,
    const Script & script,
    std::vector<std::string> & log)
{
    factory.registerBuilder<Scripted<Real>>(
        id, [&script, &log](const std::string & name, const BT::NodeConfig & config) {
            return std::make_unique<Scripted<Real>>(name, config, script, log);
        });
}

// The shipped trees on scripted leaves, run as the executor runs them.
class Missions
{
public:
    explicit Missions(Script script) : script_(std::move(script))
    {
        registerScripted<piper_bt::MoveArmToNamedPose>(
            factory_, "MoveArmToNamedPose", script_, log_);
        registerScripted<piper_bt::ViewWorkspace>(factory_, "ViewWorkspace", script_, log_);
        registerScripted<piper_bt::FindObjects>(factory_, "FindObjects", script_, log_);
        registerScripted<piper_bt::PickObject>(factory_, "PickObject", script_, log_);
        registerScripted<piper_bt::PlaceObject>(factory_, "PlaceObject", script_, log_);
        registerScripted<piper_bt::PlaceTarget>(factory_, "PlaceTarget", script_, log_);
        registerScripted<piper_bt::RunStage>(factory_, "RunStage", script_, log_);
        for (const auto & entry : std::filesystem::recursive_directory_iterator(
                 std::filesystem::path(PIPER_BT_TREES_DIR)))
        {
            if (entry.path().extension() == ".xml")
            {
                factory_.registerBehaviorTreeFromFile(entry.path());
                auto models = piper_bt::treeInputs(readFile(entry.path()));
                models_.merge(models);
            }
        }
    }

    // Runs a tree to its end, its keys checked and completed from its model as a run's are.
    BT::NodeStatus run(const std::string & id, const Entries & entries)
    {
        const auto model = models_.find(id);
        auto blackboard = BT::Blackboard::create();
        for (const auto & [key, value] : piper_bt::checkBlackboard(
                 entries, model == models_.end() ? std::nullopt : std::optional{model->second}))
        {
            blackboard->set<std::string>(key, value);
        }
        blackboard->set<bool>("arm_stuck", false);
        BT::Tree tree = factory_.createTree(id, blackboard);
        return tree.tickWhileRunning();
    }

    [[nodiscard]] long count(const std::string & name) const
    {
        return std::ranges::count(log_, name);
    }

private:
    Script script_;
    std::vector<std::string> log_;
    std::map<std::string, piper_bt::TreeInputs> models_;
    BT::BehaviorTreeFactory factory_;
};

constexpr auto kOk = BT::NodeStatus::SUCCESS;
constexpr auto kNo = BT::NodeStatus::FAILURE;

}  // namespace

TEST_F(TreeLoads, EveryShippedTreeBuilds)
{
    const auto node = executorOn(PIPER_BT_TREES_DIR);
    const auto trees = node->trees();
    for (const char * id :
         {"PickFromTable",
          "PlaceOnTable",
          "PickAndPlace",
          "SortObjects",
          "ArrangeInRow",
          "ClearWorkspace",
          "ArmToPose"})
    {
        EXPECT_NE(std::ranges::find(trees, id), trees.end()) << id;
    }
    BT::BehaviorTreeFactory factory;
    piper_bt::RosContext context;
    context.node = node.get();
    context.clients = std::make_shared<piper_bt::Clients>(node.get());
    piper_bt::registerLeaves(factory, context);
    for (const auto & entry :
         std::filesystem::recursive_directory_iterator(std::filesystem::path(PIPER_BT_TREES_DIR)))
    {
        if (entry.path().extension() == ".xml")
        {
            factory.registerBehaviorTreeFromFile(entry.path());
        }
    }
    for (const std::string & id : trees)
    {
        EXPECT_NO_THROW({ const BT::Tree tree = factory.createTree(id); }) << id;
    }
}

TEST_F(TreeLoads, RejectsALeafNobodyRegistered)
{
    const auto node = std::make_shared<rclcpp::Node>("test_unknown_leaf");
    BT::BehaviorTreeFactory factory;
    piper_bt::RosContext context;
    context.node = node.get();
    context.clients = std::make_shared<piper_bt::Clients>(node.get());
    piper_bt::registerLeaves(factory, context);
    EXPECT_THROW(
        static_cast<void>(factory.createTreeFromText(
            R"(<root BTCPP_format="4"><BehaviorTree ID="T"><NoSuchSkill/></BehaviorTree></root>)")),
        BT::RuntimeError);
}

TEST_F(TreeLoads, ThePaletteIsCurrent)
{
    const auto node = std::make_shared<rclcpp::Node>("test_palette");
    BT::BehaviorTreeFactory factory;
    piper_bt::RosContext context;
    context.node = node.get();
    context.clients = std::make_shared<piper_bt::Clients>(node.get());
    piper_bt::registerLeaves(factory, context);
    // Line ends at the end do not count.
    const auto text = [](std::string s) {
        while (!s.empty() && s.back() == '\n')
        {
            s.pop_back();
        }
        return s;
    };
    EXPECT_EQ(text(piper_bt::leafModelXml(factory)), text(readFile(PIPER_BT_PALETTE)))
        << "regenerate it: ros2 run piper_bt bt_node_model > groot/piper_bt_nodes.xml";
}

TEST_F(TreeLoads, RefusesTwoTreesOfOneIdAndNoTrees)
{
    const TempDir dir;
    dir.write(
        "a.xml",
        R"(<root BTCPP_format="4"><BehaviorTree ID="T"><AlwaysSuccess/></BehaviorTree></root>)");
    dir.write(
        "b.xml",
        R"(<root BTCPP_format="4"><BehaviorTree ID="T"><AlwaysFailure/></BehaviorTree></root>)");
    try
    {
        static_cast<void>(executorOn(dir.path().string()));
        FAIL() << "two trees named T loaded";
    }
    catch (const std::runtime_error & e)
    {
        const std::string what = e.what();
        EXPECT_NE(what.find("a.xml"), std::string::npos) << what;
        EXPECT_NE(what.find("b.xml"), std::string::npos) << what;
    }
    const TempDir empty;
    EXPECT_THROW(static_cast<void>(executorOn(empty.path().string())), std::runtime_error);
}

TEST_F(TreeLoads, ATreeModelSaysWhichKeysARunTakes)
{
    const auto inputs = piper_bt::treeInputs(
        readFile(std::filesystem::path(PIPER_BT_TREES_DIR) / "missions" / "PickAndPlace.xml"));
    ASSERT_TRUE(inputs.contains("PickAndPlace"));
    const auto & pick_and_place = inputs.at("PickAndPlace");
    EXPECT_FALSE(pick_and_place.at("object").has_value());
    EXPECT_EQ(pick_and_place.at("place_region").value_or(""), "any");

    const Entries good{{"object", "red cube"}, {"place_mode", "free"}, {"place_region", "left"}};
    const auto checked = piper_bt::checkBlackboard(good, pick_and_place);
    EXPECT_NE(
        std::ranges::find(checked, std::pair<std::string, std::string>{"pick_approach", "auto"}),
        checked.end());
    Entries misspelt = good;
    misspelt[2].first = "region";
    EXPECT_THROW(
        static_cast<void>(piper_bt::checkBlackboard(misspelt, pick_and_place)),
        std::invalid_argument);
    EXPECT_THROW(
        static_cast<void>(
            piper_bt::checkBlackboard(Entries{{"place_mode", "free"}}, pick_and_place)),
        std::invalid_argument);
    EXPECT_EQ(piper_bt::checkBlackboard(misspelt, std::nullopt).size(), misspelt.size());
}

TEST_F(TreeLoads, APickAndPlacePicksPlacesAndGoesBackToReady)
{
    std::vector<std::string> asked;
    Missions missions([&asked](const std::string & name, BT::TreeNode & leaf) {
        if (name == "pick_it")
        {
            asked.push_back(leaf.getInput<std::string>("prompt").value());
        }
        if (name == "place_it")
        {
            asked.push_back(leaf.getInput<std::string>("mode").value());
            asked.push_back(leaf.getInput<std::string>("region").value());
        }
        return kOk;
    });
    EXPECT_EQ(
        missions.run(
            "PickAndPlace",
            {{"object", "red cube"}, {"place_mode", "free"}, {"place_region", "left"}}),
        kOk);
    EXPECT_EQ(asked, (std::vector<std::string>{"red cube", "free", "left"}));
    EXPECT_EQ(missions.count("pick_it"), 1);
    EXPECT_EQ(missions.count("find_it_again"), 0);
    EXPECT_EQ(missions.count("back_to_ready"), 1);
}

TEST_F(TreeLoads, AFailedPickLooksAgainAndAllFailedBackToReady)
{
    const auto run = [](int failed_picks, long * picks, long * looks) {
        Missions missions([&failed_picks](const std::string & name, BT::TreeNode &) {
            if (name == "pick_it" && failed_picks > 0)
            {
                --failed_picks;
                return kNo;
            }
            return kOk;
        });
        const auto status = missions.run("PickAndPlace", {{"object", "red cube"}});
        *picks = missions.count("pick_it");
        *looks = missions.count("find_it_again");
        EXPECT_EQ(missions.count("place_it"), status == kOk ? 1 : 0);
        EXPECT_EQ(missions.count("back_to_ready"), 1);
        return status;
    };
    long picks = 0;
    long looks = 0;
    EXPECT_EQ(run(1, &picks, &looks), kOk);
    EXPECT_EQ(picks, 2);
    EXPECT_EQ(looks, 1);
    EXPECT_EQ(run(3, &picks, &looks), kNo);
    EXPECT_EQ(picks, 3);
    EXPECT_EQ(looks, 3);
}

TEST_F(TreeLoads, AStuckArmStopsEveryMoveForTheRestOfTheRun)
{
    Missions missions([](const std::string & name, BT::TreeNode & leaf) {
        if (name == "place_it")
        {
            piper_bt::reportArmStuck(leaf, true);
            return kNo;
        }
        return kOk;
    });
    EXPECT_EQ(missions.run("PickAndPlace", {{"object", "red cube"}}), kNo);
    EXPECT_EQ(missions.count("pick_it"), 1);
    EXPECT_EQ(missions.count("place_it"), 1);
    EXPECT_EQ(missions.count("back_to_ready"), 0);
}

TEST_F(TreeLoads, ArrangeInRowPutsEachObjectSpacingFurtherRight)
{
    std::vector<double> xs;
    Missions missions([&xs](const std::string & name, BT::TreeNode & leaf) {
        if (name == "place_it")
        {
            xs.push_back(leaf.getInput<double>("x").value());
            EXPECT_EQ(leaf.getInput<std::string>("mode").value(), "metric");
            EXPECT_NEAR(leaf.getInput<double>("y").value(), 0.05, 1e-9);
        }
        return kOk;
    });
    const Entries entries{
        {"objects", "red cube;blue cube;green cube"}, {"spacing_m", "0.1"}, {"row_y_m", "0.05"}};
    EXPECT_EQ(missions.run("ArrangeInRow", entries), kOk);
    ASSERT_EQ(xs.size(), 3U);
    EXPECT_NEAR(xs[0], -0.1, 1e-9);
    EXPECT_NEAR(xs[1], 0.0, 1e-9);
    EXPECT_NEAR(xs[2], 0.1, 1e-9);
    // The first object it cannot put down ends the row: it is still in the gripper.
    Missions stuck(
        [](const std::string & name, BT::TreeNode &) { return name == "place_it" ? kNo : kOk; });
    EXPECT_EQ(stuck.run("ArrangeInRow", entries), kNo);
    EXPECT_EQ(stuck.count("pick_it"), 1);
}

TEST_F(TreeLoads, ClearWorkspaceEndsOnWhatItLastSaw)
{
    // The outcome with `objects` on the table, picks and places ending as asked; `looks` false has
    // every look fail (perception, motion) instead of showing nothing.
    const auto clear = [](int objects,
                          BT::NodeStatus pick,
                          BT::NodeStatus place,
                          long * placed,
                          bool looks = true) {
        Missions missions(
            [&objects, pick, place, looks](const std::string & name, BT::TreeNode & leaf) {
                if (name == "look" || name == "find_it_again")
                {
                    if (objects > 0)
                    {
                        return kOk;
                    }
                    if (name == "look" && looks)
                    {
                        EXPECT_TRUE(leaf.setOutput("nothing_in_view", true).has_value());
                    }
                    return kNo;
                }
                if (name == "pick_it")
                {
                    objects -= pick == kOk ? 1 : 0;
                    return pick;
                }
                return name == "place_it" ? place : kOk;
            });
        const BT::NodeStatus status =
            missions.run("ClearWorkspace", {{"object", "cube"}, {"region", "right"}});
        *placed = missions.count("place_it");
        return status;
    };
    long placed = 0;
    EXPECT_EQ(clear(2, kOk, kOk, &placed), kOk);
    EXPECT_EQ(placed, 2);
    // Nothing there: nothing to clear.
    EXPECT_EQ(clear(0, kOk, kOk, &placed), kOk);
    EXPECT_EQ(placed, 0);
    // A look that fails says nothing about the table, after a move too.
    EXPECT_EQ(clear(0, kOk, kOk, &placed, false), kNo);
    EXPECT_EQ(clear(1, kOk, kOk, &placed, false), kNo);
    EXPECT_EQ(placed, 1);
    // An object in view that no pick takes stays there.
    EXPECT_EQ(clear(1, kNo, kOk, &placed), kNo);
    EXPECT_EQ(placed, 0);
    // An object it cannot put down: the place tried twice, and no further pick.
    EXPECT_EQ(clear(2, kOk, kNo, &placed), kNo);
    EXPECT_EQ(placed, 2);
}

TEST_F(TreeLoads, SortObjectsSkipsAnObjectNotNamed)
{
    // The second object left out when empty: the run takes the model's none.
    const auto sort = [](const std::string & second) {
        std::vector<std::string> regions;
        Missions missions([&regions](const std::string & name, BT::TreeNode & leaf) {
            if (name == "place_it")
            {
                regions.push_back(leaf.getInput<std::string>("region").value());
            }
            return kOk;
        });
        Entries entries{{"object_1", "red cube"}, {"object_3", "green cube"}};
        if (!second.empty())
        {
            entries.emplace_back("object_2", second);
        }
        EXPECT_EQ(missions.run("SortObjects", entries), kOk);
        return regions;
    };
    EXPECT_EQ(sort(""), (std::vector<std::string>{"left", "right"}));
    EXPECT_EQ(sort("blue cube"), (std::vector<std::string>{"left", "middle", "right"}));
}

TEST_F(TreeLoads, AssembleStagesRunsTheStagesInOrderUntilOneFails)
{
    std::vector<std::string> stages;
    std::string failing;
    Missions missions([&](const std::string & name, BT::TreeNode & leaf) {
        if (name == "run")
        {
            stages.push_back(leaf.getInput<std::string>("stage").value());
            return stages.back() == failing ? kNo : kOk;
        }
        return kOk;
    });
    const Entries entries{{"stages", "ring;collar;pin;cap"}};
    EXPECT_EQ(missions.run("AssembleStages", entries), kOk);
    EXPECT_EQ(stages, (std::vector<std::string>{"ring", "collar", "pin", "cap"}));
    EXPECT_EQ(missions.count("done"), 1);
    // A failed stage ends the run back at ready: the later stages never run.
    stages.clear();
    failing = "collar";
    EXPECT_EQ(missions.run("AssembleStages", entries), kNo);
    EXPECT_EQ(stages, (std::vector<std::string>{"ring", "collar"}));
    EXPECT_EQ(missions.count("back_to_ready"), 1);
}
