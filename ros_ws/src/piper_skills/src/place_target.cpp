#include "piper_skills/place_target.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "piper_skills/view_planner.hpp"

namespace piper_skills
{

namespace
{

double wrapPi(double angle_rad)
{
    return std::remainder(angle_rad, 2.0 * std::numbers::pi);
}

}  // namespace

std::optional<PlaceMode> parsePlaceMode(std::string_view text)
{
    if (text.empty() || text == "free")
    {
        return PlaceMode::kFree;
    }
    if (text == "relative")
    {
        return PlaceMode::kRelative;
    }
    if (text == "metric")
    {
        return PlaceMode::kMetric;
    }
    if (text == "anchor")
    {
        return PlaceMode::kAnchor;
    }
    return std::nullopt;
}

PlaceTarget placeTarget(
    const TableAxes & axes,
    PlaceMode mode,
    double x,
    double y,
    double inset_m,
    const std::optional<Eigen::Vector3d> & anchor)
{
    if (!std::isfinite(x) || !std::isfinite(y))
    {
        return {.point = std::nullopt, .why = "x and y must be finite"};
    }
    // How far from the centre the object's centre may go and still sit fully on the top.
    const double reach_x_m = std::max(0.0, axes.half_x_m - inset_m);
    const double reach_y_m = std::max(0.0, axes.half_y_m - inset_m);
    Eigen::Vector3d point;
    switch (mode)
    {
        case PlaceMode::kFree:
            return {.point = std::nullopt, .why = "the free mode has no target"};
        case PlaceMode::kRelative:
            if (std::abs(x) > 1.0 || std::abs(y) > 1.0)
            {
                return {
                    .point = std::nullopt,
                    .why = std::format("relative x and y go from -1 to 1, not ({}, {})", x, y)};
            }
            return {
                .point =
                    axes.origin + (x * reach_x_m * axes.x_axis) + (y * reach_y_m * axes.y_axis),
                .why = {}};
        case PlaceMode::kMetric:
            point = axes.origin + (x * axes.x_axis) + (y * axes.y_axis);
            break;
        case PlaceMode::kAnchor:
            if (!anchor)
            {
                return {.point = std::nullopt, .why = "the anchor was not found"};
            }
            point = *anchor + (x * axes.x_axis) + (y * axes.y_axis);
            point.z() = axes.origin.z();
            break;
    }
    const Eigen::Vector3d d = point - axes.origin;
    const double along_x = d.dot(axes.x_axis);
    const double along_y = d.dot(axes.y_axis);
    if (std::abs(along_x) > reach_x_m + 1e-9 || std::abs(along_y) > reach_y_m + 1e-9)
    {
        return {
            .point = std::nullopt,
            .why = std::format(
                "the target ({:.3f}, {:.3f}) m from the top's centre is off the top: the object "
                "fits within +-{:.3f} m and +-{:.3f} m",
                along_x,
                along_y,
                reach_x_m,
                reach_y_m)};
    }
    return {.point = point, .why = {}};
}

TopRegion targetArea(
    const TopRegion & top, const Eigen::Vector3d & target, double tolerance_m, double free_radius_m)
{
    TopRegion area = top;
    area.center = target;
    area.center.z() = top.center.z();
    area.half_width_m = std::max(0.0, tolerance_m) + free_radius_m;
    area.half_depth_m = area.half_width_m;
    return area;
}

std::vector<double> turnsForYaw(
    double object_yaw_rad, double wanted_yaw_rad, double tolerance_rad, double step_rad)
{
    std::vector<std::pair<double, double>> ranked;
    const int steps =
        step_rad > 0.0 ? static_cast<int>(std::floor((tolerance_rad / step_rad) + 1e-9)) : 0;
    for (int i = -steps; i <= steps; ++i)
    {
        const double deviation_rad = i * step_rad;
        for (const double half_turn : {0.0, std::numbers::pi})
        {
            const double turn = wrapPi(wanted_yaw_rad + deviation_rad - object_yaw_rad + half_turn);
            ranked.emplace_back(std::abs(deviation_rad), turn);
        }
    }
    std::ranges::stable_sort(ranked, [](const auto & a, const auto & b) {
        return a.first != b.first ? a.first < b.first : std::abs(a.second) < std::abs(b.second);
    });
    std::vector<double> turns;
    turns.reserve(ranked.size());
    for (const auto & r : ranked)
    {
        turns.push_back(r.second);
    }
    return turns;
}

std::vector<double> turnsAfter(const std::vector<double> & turns, double failed_rad)
{
    const auto it = std::ranges::find(turns, failed_rad);
    return it == turns.end() ? std::vector<double>{} : std::vector<double>(it + 1, turns.end());
}

}  // namespace piper_skills
