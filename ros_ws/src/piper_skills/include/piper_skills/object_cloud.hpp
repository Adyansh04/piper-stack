#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace piper_skills
{

/** @brief Pinhole intrinsics of the depth-aligned colour camera. */
struct CameraModel
{
    /// Focal lengths and principal point, pixels.
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
};

/** @brief How a mask becomes an object's points. */
struct CloudOptions
{
    /// Depth readings outside this range are not used, metres.
    double min_depth_m{0.1};
    double max_depth_m{2.0};
    /// The mask shrinks by this many pixels first: its edge mixes object and background.
    int erode_px{2};
    /// Points farther than this from the mask's median depth are background seen through it, metres.
    double depth_band_m{0.15};
};

/**
 * @brief The points of one instance mask, in the camera's optical frame.
 * @param depth_m CV_32FC1 depth aligned with the colour image, metres (0 or NaN: no reading).
 * @param mask CV_8UC1 mask of the colour image's size, non-zero inside the instance.
 * @param camera Intrinsics of the colour image.
 * @param options Range, erosion and depth band.
 * @return The points, metres; empty when the mask has no valid depth.
 */
[[nodiscard]] std::vector<Eigen::Vector3f> maskPoints(
    const cv::Mat & depth_m,
    const cv::Mat & mask,
    const CameraModel & camera,
    const CloudOptions & options);

/**
 * @brief The points of every `step_px`-th pixel of a depth image, in the camera's optical frame.
 * @param depth_m CV_32FC1 depth, metres (0 or NaN: no reading).
 * @param camera Intrinsics of the image.
 * @param step_px Pixel step along rows and columns; below 1 counts as 1.
 * @param min_depth_m Readings nearer than this are left out, metres.
 * @param max_depth_m Readings farther than this are left out, metres.
 * @return The points, metres.
 */
[[nodiscard]] std::vector<Eigen::Vector3f> depthPoints(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    int step_px,
    double min_depth_m,
    double max_depth_m);

/**
 * @brief An object's own points among those of its mask: above the top it stands on, and in the
 * largest cluster.
 * @details A mask that spills onto the top or a neighbour brings their points along. The slice just
 * above the top surface goes first; the rest falls into clusters of touching `gap_m` voxels (faces,
 * edges or corners), and the largest one stays.
 * @param points Points in a z-up frame, metres.
 * @param top_z_m Height of the top surface, metres; NaN keeps every height.
 * @param above_top_m Points up to this far above the top surface are the top's, metres.
 * @param gap_m Voxel size of the clustering, metres; 0 or less keeps every point above the top.
 * @return The kept points; empty when none is above the top.
 */
[[nodiscard]] std::vector<Eigen::Vector3d> ownPoints(
    const std::vector<Eigen::Vector3d> & points, double top_z_m, double above_top_m, double gap_m);

/** @brief Where an object is and how large, from its points. */
struct ObjectShape
{
    /// Centroid of the points, metres.
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    /// Axes: x and y the principal directions of the points on the floor, z up.
    Eigen::Matrix3d axes{Eigen::Matrix3d::Identity()};
    /// Extent of the points along the axes, metres.
    Eigen::Vector3d size{Eigen::Vector3d::Zero()};
};

/**
 * @brief Centroid, floor-plane principal axes and extents of points in a z-up frame.
 * @param points The object's points, at least one.
 * @return The shape.
 */
[[nodiscard]] ObjectShape describe(const std::vector<Eigen::Vector3d> & points);

/**
 * @brief Draws one instance mask on an image: the mask tinted half in a colour, its box outlined and
 * a label above it.
 * @param image CV_8UC3 image to draw on, in the channel order of `colour`.
 * @param mask CV_8UC1 mask of the box's size, non-zero inside the instance.
 * @param box Where the mask lies in the image; the part outside the image is left out.
 * @param colour Tint, outline and label colour.
 * @param label Text above the box, or inside it where the box touches the image's top.
 */
void drawInstance(
    cv::Mat & image,
    const cv::Mat & mask,
    const cv::Rect & box,
    const cv::Scalar & colour,
    const std::string & label);

}  // namespace piper_skills
