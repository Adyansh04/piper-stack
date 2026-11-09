#include <gtest/gtest.h>

#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "piper_skills/task_stages.hpp"

namespace piper_skills
{
namespace
{

constexpr const char * kTask = R"(
scene:
  start:
    ring: {in: fixture, xyz: [-0.08, -0.042, 0.007], rpy: [0, 0, 180]}
  goal:
    ring: {in: base, xyz: [-0.035, 0.0, 0.012], rpy: [0, 0, 180]}
    collar: {in: base, xyz: [0.035, 0.0, 0.012], rpy: [0, 0, -90]}
    cap: {in: collar, xyz: [0.0, 0.0, 0.008], rpy: [0, 0, -30]}
stages:
  - name: ring
    part: ring
    grasp: {xyz: [0.0, 0.0, 0.0105], rpy: [180, 0, 90], opening_m: 0.022, symmetry: 2}
    approach_m: 0.035
    release_m: 0.001
    motion: insert
    contacts: [[ring, base]]
  - name: collar
    part: collar
    grasp: {xyz: [0.0, 0.0, 0.012], rpy: [180, 0, 90], opening_m: 0.025}
    approach_m: 0.03
    release_m: 0.0
    motion: screw
    screw: {pitch_m: 0.0035, turns: 2.5}
  - name: cap
    part: cap
    grasp: {xyz: [0.0, 0.0, 0.011], rpy: [180, 0, 90], opening_m: 0.03}
    approach_m: 0.03
    release_m: 0.0
    motion: twist
    twist_deg: -30
)";

std::string withStage(const std::string & stage)
{
    return "scene:\n  goal:\n    ring: {in: base}\nstages:\n" + stage;
}

TEST(TaskStages, ReadsEachStageWithItsGoalAndMate)
{
    const auto stages = parseStages(kTask);
    ASSERT_EQ(stages.size(), 3U);
    EXPECT_EQ(stages[0].name, "ring");
    EXPECT_EQ(stages[0].mate, "base");
    EXPECT_EQ(stages[0].support, "fixture");
    EXPECT_EQ(stages[0].motion, StageMotion::kInsert);
    EXPECT_NEAR(stages[0].goal.translation().x(), -0.035, 1e-12);
    EXPECT_EQ(stages[0].symmetry, 2);
    ASSERT_EQ(stages[0].contacts.size(), 1U);
    EXPECT_EQ(stages[0].contacts[0].second, "base");
    EXPECT_EQ(stages[1].motion, StageMotion::kScrew);
    EXPECT_EQ(stages[1].symmetry, 1);
    EXPECT_TRUE(stages[1].support.empty());
    EXPECT_EQ(stages[2].mate, "collar");
    EXPECT_NEAR(stages[2].twist_rad, -std::numbers::pi / 6.0, 1e-12);
}

TEST(TaskStages, GraspPointsTheGripperDownWithTheFingersAcrossTheFlats)
{
    const Stage ring = parseStages(kTask)[0];
    // The fingers close along the TCP's y: across the part's x, where its flats face.
    EXPECT_TRUE(ring.grasp.linear().col(1).isApprox(Eigen::Vector3d::UnitX(), 1e-12));
    EXPECT_TRUE(ring.grasp.linear().col(2).isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    EXPECT_NEAR(ring.grasp.translation().z(), 0.0105, 1e-12);
    const auto grasps = stageGrasps(ring);
    ASSERT_EQ(grasps.size(), 2U);
    EXPECT_TRUE(grasps[1].linear().col(1).isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
    EXPECT_TRUE(grasps[1].translation().isApprox(ring.grasp.translation(), 1e-12));
}

TEST(TaskStages, ScrewStartsTurnsAboveTheSeatTurnedBack)
{
    const Stage collar = parseStages(kTask)[1];
    EXPECT_NEAR(stageTurn(collar), -5.0 * std::numbers::pi, 1e-12);
    const Eigen::Isometry3d start = partOnMate(collar, 1.0);
    EXPECT_NEAR(start.translation().z(), 0.012 + 2.5 * 0.0035, 1e-12);  // 8.75 mm up
    // 2.5 turns back from a yaw of -90 degrees: +90 degrees.
    EXPECT_TRUE(start.linear().col(0).isApprox(Eigen::Vector3d::UnitY(), 1e-9));
    const Eigen::Isometry3d half = partOnMate(collar, 0.5);
    EXPECT_NEAR(half.translation().z(), 0.012 + 1.25 * 0.0035, 1e-12);
    EXPECT_TRUE(partOnMate(collar, 0.0).isApprox(collar.goal, 1e-12));
}

TEST(TaskStages, TwistStartsTurnedBackAtTheGoalRaisedByTheRelease)
{
    Stage cap = parseStages(kTask)[2];
    cap.release_m = 0.0005;
    const Eigen::Isometry3d start = partOnMate(cap, 1.0);
    // Turned back by 30 degrees from -30: the entry, at yaw 0 in the collar's frame.
    EXPECT_TRUE(start.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
    EXPECT_NEAR(start.translation().z(), 0.0085, 1e-12);
    EXPECT_NEAR(partOnMate(cap, 0.0).translation().z(), 0.0085, 1e-12);
}

TEST(TaskStages, AStageComesInAlongItsApproachAxis)
{
    const std::string plug =
        "scene:\n  goal:\n    plug: {in: socket, xyz: [0, 0, -0.0066], rpy: [180, 0, 0]}\n"
        "stages:\n  - {name: plug, part: plug, grasp: {opening_m: 0.012}, approach_m: 0.02,\n"
        "     approach_axis: [0, 0, -2], release_m: 0.001, motion: insert}\n";
    const Stage stage = parseStages(plug)[0];
    EXPECT_TRUE(stage.approach_axis.isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
    // Let go 1 mm back along -z of the plug, which points into the socket: out of it.
    const Eigen::Isometry3d released = partOnMate(stage, 0.0);
    EXPECT_NEAR(released.translation().z(), -0.0066 + 0.001, 1e-12);
    EXPECT_THROW(
        (void)parseStages(
            "scene:\n  goal:\n    ring: {in: base}\nstages:\n  - {name: ring, part: ring, "
            "grasp: {opening_m: 0.02}, approach_m: 0.03, release_m: 0.0, motion: insert, "
            "approach_axis: [0, 0, 0]}\n"),
        std::invalid_argument);
}

TEST(TaskStages, RefusesBadStages)
{
    const std::string ring =
        "  - {name: ring, part: ring, grasp: {opening_m: 0.02}, approach_m: 0.03, release_m: 0.0";
    EXPECT_NO_THROW((void)parseStages(withStage(ring + ", motion: insert}")));
    EXPECT_THROW((void)parseStages(withStage(ring + ", motion: press}")), std::invalid_argument);
    EXPECT_THROW((void)parseStages(withStage(ring + ", motion: screw}")), std::invalid_argument);
    EXPECT_THROW((void)parseStages(withStage(ring + ", motion: twist}")), std::invalid_argument);
    EXPECT_THROW(
        (void)parseStages(withStage(ring + ", motion: insert}\n" + ring + ", motion: insert}")),
        std::invalid_argument);
    const std::string pin =
        "  - {name: pin, part: pin, grasp: {opening_m: 0.01}, approach_m: 0.03, release_m: 0.0, "
        "motion: insert}";
    EXPECT_THROW((void)parseStages(withStage(pin)), std::invalid_argument);
    EXPECT_THROW((void)parseStages("stages: []"), std::invalid_argument);
    EXPECT_THROW((void)parseStages("stages: [unclosed"), std::invalid_argument);
}

}  // namespace
}  // namespace piper_skills
