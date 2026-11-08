#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace piper_skills
{

/** @brief A rectangle on the table's top, in the robot's base frame (z up). */
struct TopRegion
{
    /// Centre, metres.
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    /// Unit vectors on the floor: across (the robot's left) and away from the robot.
    Eigen::Vector3d lateral{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d forward{Eigen::Vector3d::UnitX()};
    /// Half extents along `lateral` and `forward`, metres.
    double half_width_m{0.5};
    double half_depth_m{0.4};
};

/** @brief Camera and search options of the view planner. */
struct ViewConfig
{
    /// Full fields of view, radians.
    // The YAML's defaults: 69 and 42.3 degrees.
    double hfov_rad{1.2043};
    double vfov_rad{0.7383};
    /// Share of the image kept free around the region on every side.
    double image_margin{0.05};
    /// Share of the image's height at its bottom the gripper hides (fingers and wrist in view).
    double occluded_bottom{0.1};
    /// Camera distances searched from the region's centre, metres.
    double min_distance_m{0.35};
    double max_distance_m{1.2};
    /// Tilts tried, from looking straight down towards the robot's view of the top, radians.
    std::vector<double> tilts_rad{0.0, 0.2618, 0.5236, 0.7854, 1.0472};
    /// Views cover the top at most this far from its near edge, the band the arm reaches, metres
    /// (0: the whole depth).
    double max_depth_m{0.5};
    /// Sections are at most this wide along the side the robot faces, metres; at most this many.
    double section_max_width_m{0.7};
    int max_sections{5};
    /// Sections overlap their neighbours by this share of their width.
    double section_overlap{0.15};
};

/** @brief One way to view a region: the camera's optical frame (z forward, x right, y down). */
struct ViewCandidate
{
    /// The camera's optical frame in the base frame.
    Eigen::Isometry3d camera{Eigen::Isometry3d::Identity()};
    /// Its tilt from looking straight down, radians.
    double tilt_rad{0.0};
    /// Its distance from the region's centre, metres.
    double distance_m{0.0};
};

/** @brief A named view of the table's top with its candidates, best first. */
struct TopView
{
    /// "full", then "left", "middle", "right" (as the robot sees them) or "section_<k>".
    std::string name;
    /// The part of the top the view shows, base frame.
    TopRegion region;
    /// Camera poses that show it, best first.
    std::vector<ViewCandidate> candidates;
};

/**
 * @brief Camera poses that show a region whole: per tilt, the closest camera looking at the
 * region's centre that keeps its four corners inside the image.
 * @param region The region.
 * @param config Camera and search options.
 * @return Candidates in the order of `config.tilts_rad`; empty when no distance in range fits.
 */
[[nodiscard]] std::vector<ViewCandidate> viewRegion(
    const TopRegion & region, const ViewConfig & config);

/**
 * @brief The full view of the table's top and, when it is wider than `section_max_width_m`, the
 * section views across it, all within `max_depth_m` of its near edge.
 * @param whole The whole top.
 * @param config Camera and search options.
 * @return "full" first, then the sections from the robot's left to its right.
 */
[[nodiscard]] std::vector<TopView> planTopViews(const TopRegion & whole, const ViewConfig & config);

/**
 * @brief Whether a point is inside a camera's view.
 * @param camera Optical frame in the base frame.
 * @param point Point in the base frame, metres.
 * @param config Fields of view and margin.
 * @return True when in front of the camera and inside the image less its margin.
 */
[[nodiscard]] bool inView(
    const Eigen::Isometry3d & camera, const Eigen::Vector3d & point, const ViewConfig & config);

}  // namespace piper_skills
