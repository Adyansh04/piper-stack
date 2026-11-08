#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "piper_skills/object_cloud.hpp"
#include "piper_skills/view_planner.hpp"

namespace piper_skills
{

/** @brief How free table is told from occupied table in one depth image. */
struct FreeSpotOptions
{
    /// A spot is free when nothing stands within this radius of it, metres.
    double free_radius_m{0.12};
    /// Candidate spots lie on a grid of this step, metres.
    double grid_step_m{0.03};
    /// Cost weight of the distance from the area's near edge (nearer the robot is easier to reach).
    double near_weight{0.5};
    /// Spots farther than this from the base frame's z axis are out of the arm's reach from above,
    /// metres; 0: no limit.
    double max_reach_m{0.0};
    /// Every stride_px-th pixel of the depth image is looked at.
    int stride_px{2};
    /// Points higher than this above the top stand on it, metres.
    double occupied_above_m{0.01};
    /// Points nearer the camera than this are the gripper or the held object, metres.
    double min_camera_distance_m{0.35};
    /// Points up to this height above the top take a spot: what stands on the top and what hangs
    /// over it, which the held object and the gripper would meet coming down, metres.
    double max_object_height_m{0.6};
    /// Points higher than wrist_above_m above the top are where the wrist and its camera come down
    /// beside the gripper: they take a spot within wrist_radius_m, metres.
    double wrist_above_m{0.2};
    double wrist_radius_m{0.22};
    /// So many points within a radius take a spot: one bad depth pixel does not.
    int min_occupied_points{1};
    /// A depth reading this far from the top's plane at a circle's sample is not the top: farther
    /// is past the top's real edge, nearer is something hiding it, metres.
    double surface_tolerance_m{0.03};
    /// Within under_radius_m of a spot (what the held object covers) a spot is refused when more
    /// than this share of its samples has no depth or is not the top; 0 radius: no such rule.
    double under_radius_m{0.0};
    double max_no_depth_share_under{0.1};
    /// The TCP when the image was taken, base frame. Points within held_radius_m of its vertical
    /// and no lower than held_below_m under it are the gripper and the held object.
    std::optional<Eigen::Vector3d> tcp;
    double held_radius_m{0.0};
    double held_below_m{0.0};
    /// Points higher than tall_above_m above the top take a spot within tall_radius_m when that is
    /// more than free_radius_m, and its whole circle must be in view; 0: no such rule, metres.
    double tall_above_m{0.12};
    double tall_radius_m{0.0};
    /// A spot is refused when more than this share of its circle has no depth.
    double max_no_depth_share{0.3};
    /// Depth readings outside this range are not used, metres.
    double min_depth_m{0.1};
    double max_depth_m{2.0};
};

/** @brief Why the candidate spots of one free-spot search were refused. */
struct FreeSpotCounts
{
    /// Candidates looked at: on the top and cheaper than the best spot so far.
    int checked{0};
    /// Their circle not wholly in the view (out of the image or behind the gripper).
    int unseen{0};
    /// A point standing on the top within the free radius.
    int taken{0};
    /// More than max_no_depth_share of the circle without depth.
    int no_depth{0};
    /// Grid points on the top beyond max_reach_m, not looked at.
    int out_of_reach{0};
};

/** @brief The outcome of a free-spot search. */
struct FreeSpotResult
{
    /// The spot on the top, base frame, metres; nullopt when none is free.
    std::optional<Eigen::Vector3d> spot;
    /// The points found standing on the top, base frame, metres.
    std::vector<Eigen::Vector3d> occupied;
    /// Why candidates were refused; every candidate on the top is counted when none is free.
    FreeSpotCounts counts;
};

/**
 * @brief The height of a table's top as a depth image shows it.
 * @details The most common height of the image's points over the top's rectangle, within `search_m`
 * of the model's: the table wins over what stands on it. The model's height is the mesh's, the
 * camera's frame is the CAD's, and the place's margins are a centimetre.
 * @param depth_m Depth image, CV_32FC1 metres.
 * @param camera Its intrinsics.
 * @param base_to_camera The camera's optical frame in the base frame.
 * @param top The top, with the model's height.
 * @param search_m How far from the model's height to look, metres.
 * @return The height in the base frame, metres; nullopt when too little of the top is in view.
 */
[[nodiscard]] std::optional<double> topHeightInView(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const TopRegion & top,
    double search_m);

/**
 * @brief The free spot of an area of a table's top in one depth image, nearest the area's centre
 * and the robot.
 * @details Spots lie on a grid centred on the area's centre. The whole circle of `free_radius_m`
 * around a spot must be on the top and in the view, no point standing on the top may be inside it,
 * and most of it must have depth.
 * @param depth_m CV_32FC1 depth aligned with the camera model, metres (0 or NaN: no reading).
 * @param camera Intrinsics of the depth image.
 * @param base_to_camera The camera's optical frame in the base frame.
 * @param top The table's whole top in the base frame.
 * @param area The part of the top to search, in the base frame.
 * @param view The camera's fields of view and margins.
 * @param options Free-space rules.
 * @return The spot, the points standing on the top and why the other candidates were refused.
 */
[[nodiscard]] FreeSpotResult freeSpot(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const TopRegion & top,
    const TopRegion & area,
    const ViewConfig & view,
    const FreeSpotOptions & options);

/**
 * @brief The middle of points' extent along a pose's axes.
 * @param pose Centroid and axes of the points.
 * @param points The points, same frame.
 * @return The middle of the extent; the pose's position when there are no points.
 */
[[nodiscard]] Eigen::Vector3d boxCentre(
    const Eigen::Isometry3d & pose, const std::vector<Eigen::Vector3d> & points);

/** @brief An object's points seen from above. */
struct Footprint
{
    /// Centroid on the floor, metres.
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
    /// Height of the highest point, metres; -1e9 without points.
    double top_z_m{-1.0e9};
    /// Largest horizontal distance of a point from the centroid, metres.
    double radius_m{0.0};
    /// Unit direction of the widest horizontal spread.
    Eigen::Vector2d long_axis{Eigen::Vector2d::UnitX()};
    /// Extents along and across `long_axis`, metres.
    double length_m{0.0};
    double breadth_m{0.0};
};

/**
 * @brief Centroid, highest point, horizontal reach and principal axes of an object's points.
 * @param points Points in a z-up frame, metres.
 * @return The footprint; the default one for no points.
 */
[[nodiscard]] Footprint footprintOf(const std::vector<Eigen::Vector3d> & points);

/**
 * @brief The TCP pose that puts a held object down with its centre on a spot.
 * @details The object keeps the attitude it was picked in, turned about the vertical by `turn_rad`.
 * @param spot Where the object's centre goes on the top, metres.
 * @param turn_rad Turn about the vertical from the attitude at the pick, radians.
 * @param tcp_rotation The TCP's rotation at the grasp.
 * @param centre_to_tcp From the object's centre to the TCP at the grasp, metres.
 * @param tcp_above_bottom_m How far the TCP was above the object's bottom at the grasp, metres.
 * @param release_height_m The object's bottom is released this far above the top, metres.
 * @return The TCP pose at release, same frame as `spot`.
 */
[[nodiscard]] Eigen::Isometry3d placeTcp(
    const Eigen::Vector3d & spot,
    double turn_rad,
    const Eigen::Matrix3d & tcp_rotation,
    const Eigen::Vector3d & centre_to_tcp,
    double tcp_above_bottom_m,
    double release_height_m);

/**
 * @brief The part of a top a place asks for, as the robot faces it.
 * @param top The whole top.
 * @param region "any" (the whole top), "left", "middle" or "right" (thirds across it).
 * @return The area; nullopt for an unknown region.
 */
[[nodiscard]] std::optional<TopRegion> placeArea(const TopRegion & top, const std::string & region);

/**
 * @brief The views to try, by name.
 * @param planned The views the planner made for the top.
 * @param asked The goal's views; when empty, `preferred` orders the planned ones and the rest follow
 * in the planner's order.
 * @param preferred The configured order.
 * @return The asked (or preferred) names that exist, then, when nothing was asked, the others.
 */
[[nodiscard]] std::vector<std::string> viewOrder(
    const std::vector<TopView> & planned,
    const std::vector<std::string> & asked,
    const std::vector<std::string> & preferred);

}  // namespace piper_skills
