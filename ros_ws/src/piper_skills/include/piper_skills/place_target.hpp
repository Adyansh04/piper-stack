#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include "piper_skills/view_planner.hpp"

namespace piper_skills
{

/** @brief How a place goal gives its target. */
enum class PlaceMode : std::uint8_t
{
    /// The free spot nearest the robot in the goal's region.
    kFree,
    /// x and y from -1 to 1 across the top: as far as the object still sits fully on it.
    kRelative,
    /// x and y in metres from the top's centre.
    kMetric,
    /// x and y in metres from an anchor object's centre on the top.
    kAnchor,
};

/**
 * @brief A place mode from its name.
 * @param text "free" (also when empty), "relative", "metric" or "anchor".
 * @return The mode; nullopt for any other text.
 */
[[nodiscard]] std::optional<PlaceMode> parsePlaceMode(std::string_view text);

/**
 * @brief The axes place targets are given in, fixed to the table's top.
 * @details As the robot faces the table: x to its right, y away from it, the origin at the top's
 * centre.
 */
struct TableAxes
{
    /// Centre of the top, metres.
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    /// Unit vectors on the floor.
    Eigen::Vector3d x_axis{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d y_axis{-Eigen::Vector3d::UnitX()};
    /// Half extents along `x_axis` and `y_axis`, metres.
    double half_x_m{0.5};
    double half_y_m{0.4};
};

/** @brief Where a placed object's centre goes, or why there is no such place. */
struct PlaceTarget
{
    std::optional<Eigen::Vector3d> point;
    std::string why;
};

/**
 * @brief The point a placed object's centre goes to.
 * @param axes The top's axes.
 * @param mode kRelative, kMetric or kAnchor (kFree has no target).
 * @param x Along `axes.x_axis`: from -1 to 1 (kRelative) or metres.
 * @param y Along `axes.y_axis`: from -1 to 1 (kRelative) or metres.
 * @param inset_m How far the object's centre must stay inside the top's edges, metres.
 * @param anchor The anchor object's centre (kAnchor), same frame as `axes`.
 * @return The point on the top, or the reason: relative values outside -1 to 1, a point off the
 * top (inset included), no anchor, or kFree.
 */
[[nodiscard]] PlaceTarget placeTarget(
    const TableAxes & axes,
    PlaceMode mode,
    double x,
    double y,
    double inset_m,
    const std::optional<Eigen::Vector3d> & anchor);

/**
 * @brief The square around a target that a free spot is searched in.
 * @param top The whole top (its axes orient the square).
 * @param target The target on the top.
 * @param tolerance_m How far from the target a spot may be, metres.
 * @param free_radius_m The free circle's radius around a spot, metres.
 * @return The area: spots within `tolerance_m` of the target along each axis.
 */
[[nodiscard]] TopRegion targetArea(
    const TopRegion & top,
    const Eigen::Vector3d & target,
    double tolerance_m,
    double free_radius_m);

/**
 * @brief The turns about the vertical that give a placed object a wanted yaw.
 * @details An object's long axis looks the same after half a turn, so both ways are offered.
 * @param object_yaw_rad The object's long-axis yaw as held, radians.
 * @param wanted_yaw_rad The wanted long-axis yaw, same frame, radians.
 * @param tolerance_rad How far the placed yaw may be off, radians.
 * @param step_rad Steps tried within the tolerance, radians.
 * @return Turns, wrapped to [-pi, pi): the smallest deviation first, then the smallest turn.
 */
[[nodiscard]] std::vector<double> turnsForYaw(
    double object_yaw_rad, double wanted_yaw_rad, double tolerance_rad, double step_rad);

/**
 * @brief The turns still to try after one that failed.
 * @param turns The turns in the order they are tried, radians.
 * @param failed_rad The turn that failed, one of `turns`, radians.
 * @return The turns after `failed_rad`, in order; empty when it is the last or not among them.
 */
[[nodiscard]] std::vector<double> turnsAfter(const std::vector<double> & turns, double failed_rad);

}  // namespace piper_skills
