#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace piper_skills
{

/** @brief How a stage puts its part on its mate. */
enum class StageMotion : std::uint8_t
{
    /// Straight down onto the goal.
    kInsert,
    /// Turned down a right-hand thread to the goal, regrasped every half turn.
    kScrew,
    /// Inserted turned back by the twist, then turned by it about the goal's z.
    kTwist,
};

/**
 * @brief How an insert goes the last of the way in moves from where the part is seen.
 * @details At `from_m` from its goal, out of contact, the part is centred and turned onto the
 * goal's axis from where it is seen, the grasp taken as the fingers hold it then, until it is
 * within `align_m` and `align_rad`. Each move then goes `step_m` deeper and presses `push_m`
 * further. A move that gains less than half a step backs out to `from_m`, aligns again and comes
 * in at the next point of a spiral round the axis, up to `spiral_m` off it; after `tries` of those
 * the stage fails. Metres and radians.
 */
struct InsertGuide
{
    /// How far from its goal, along the approach axis, the part is when the moves start.
    double from_m{0.0};
    double step_m{0.0};
    double push_m{0.0};
    double align_m{0.0};
    double align_rad{0.0};
    double spiral_m{0.0};
    int tries{0};
};

/**
 * @brief One stage of a task: a part taken where it lies and put at its goal on its mate.
 * @details Lengths in metres, angles in radians; poses as in the task's `task.yaml`.
 */
struct Stage
{
    std::string name;
    std::string part;
    /// The part whose frame the goal is in.
    std::string mate;
    /// The part it rests on before the stage (the frame of its start pose), or empty.
    std::string support;
    /// The part's goal pose in the mate's frame.
    Eigen::Isometry3d goal{Eigen::Isometry3d::Identity()};
    /// The TCP's pose in the part's frame while the fingers hold it.
    Eigen::Isometry3d grasp{Eigen::Isometry3d::Identity()};
    /// Each finger's opening from closed before it closes on the part.
    double opening_m{0.0};
    /// 1, or 2 when the grasp also holds the part turned half a turn about its z.
    int symmetry{1};
    /// How far from where its motion starts the part comes in from, along the approach axis.
    double approach_m{0.0};
    /// How far above the motion's path the part moves and is let go, along the approach axis.
    double release_m{0.0};
    /// The way, in the part's frame, it comes onto its mate from and backs off along: a unit vector,
    /// +z unless the stage says otherwise (a plug whose z points into its socket comes from -z).
    Eigen::Vector3d approach_axis{Eigen::Vector3d::UnitZ()};
    StageMotion motion{StageMotion::kInsert};
    /// Screw: the thread's pitch, and the turns from where the screw starts to the seat.
    double pitch_m{0.0};
    double turns{0.0};
    /// Twist: the turn about the goal's z; negative is clockwise seen from above.
    double twist_rad{0.0};
    /// Insert: the moves of its last part; unset, it goes straight to the goal.
    std::optional<InsertGuide> guide;
    /// How far from its goal the part may end, metres and radians; 0: the runner's defaults.
    double goal_tolerance_m{0.0};
    double goal_tolerance_rad{0.0};
    /// Pairs of parts and robot links the stage lets touch.
    std::vector<std::pair<std::string, std::string>> contacts;
};

/**
 * @brief The stages of a task, in order.
 * @param[in] task_yaml The text of the task's `task.yaml`.
 * @return Each stage, its goal taken from `scene.goal` and its support from `scene.start`.
 * @throws std::invalid_argument If the text is no YAML, a stage lacks a key or has a value out of
 * range, two stages share a name, or a stage's part has no goal relative to another part.
 */
[[nodiscard]] std::vector<Stage> parseStages(std::string_view task_yaml);

/**
 * @brief The whole turn of a stage's motion about the goal's z.
 * @param[in] stage The stage.
 * @return Radians, negative clockwise seen from above: minus the screw's turns, the twist, or 0.
 */
[[nodiscard]] double stageTurn(const Stage & stage);

/**
 * @brief Where the part is on its mate with part of its motion still to go.
 * @details The motion turns the part about the goal's z and, for a screw, lowers it one pitch per
 * turn; the path is moved by `release_m` along the approach axis throughout.
 * @param[in] stage The stage.
 * @param[in] left The share of the motion still to go: 1 where it starts, 0 at its end.
 * @return The part's pose in the mate's frame.
 */
[[nodiscard]] Eigen::Isometry3d partOnMate(const Stage & stage, double left);

/**
 * @brief Where a screw puts its part next, from where it stands on its seat.
 * @details The part turns `step_rad` down the thread (clockwise seen from above) and is pressed
 * `preload_m` below the height the thread gives it after the turn, but not below its seat by more
 * than that: the thread, not the arm, sets how far it goes down.
 * @param[in] stage A screw stage.
 * @param[in] on_seat The part upright on its seat's axis: its height above the seat and its turn.
 * @param[in] step_rad The turn, radians.
 * @param[in] preload_m How far below the thread's height the part is pressed, metres.
 * @return The part's pose in its seat's frame.
 */
[[nodiscard]] Eigen::Isometry3d screwStep(
    const Stage & stage, const Eigen::Isometry3d & on_seat, double step_rad, double preload_m);

/**
 * @brief Where a guided insert comes in again on a try after it stuck.
 * @details The points of a spiral round the approach axis, a golden angle apart, out to
 * `guide.spiral_m` at the last try.
 * @param[in] stage An insert stage with a guide.
 * @param[in] guide The stage's guide.
 * @param[in] attempt The try, 1 to `guide.tries`.
 * @return The offset across the approach axis, in the goal's frame, metres.
 */
[[nodiscard]] Eigen::Vector3d searchOffset(
    const Stage & stage, const InsertGuide & guide, int attempt);

/**
 * @brief The grasps that hold a stage's part: the grasp, turned by each step of its symmetry.
 * @param[in] stage The stage.
 * @return TCP poses in the part's frame, the stage's own grasp first.
 */
[[nodiscard]] std::vector<Eigen::Isometry3d> stageGrasps(const Stage & stage);

}  // namespace piper_skills
