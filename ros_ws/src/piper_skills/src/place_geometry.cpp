#include "piper_skills/place_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "piper_skills/object_cloud.hpp"
#include "piper_skills/view_planner.hpp"

namespace piper_skills
{
namespace
{

// Whether a point in the base frame is the gripper or the held object, in the image as it was taken.
bool isHeld(const Eigen::Vector3d & p, const FreeSpotOptions & o)
{
    return o.tcp && (p - *o.tcp).head<2>().norm() < o.held_radius_m
           && p.z() > o.tcp->z() - o.held_below_m;
}

// Whether a spot's circle shows the top: enough of its samples read the top's plane. A sample out
// of the image, without depth, far behind the plane (the floor past the top's real edge) or far in
// front of it (hidden by something taller) does not, unless the gripper hides it.
// NOLINTNEXTLINE(readability-function-cognitive-complexity): one pass over the image's points.
bool topInView(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const Eigen::Isometry3d & camera_from_base,
    const TopRegion & area,
    const Eigen::Vector3d & centre,
    const FreeSpotOptions & o)
{
    int missing = 0;
    int samples = 0;
    int missing_under = 0;
    int samples_under = 0;
    const auto count = [&](bool bad, double from_centre_m) {
        ++samples;
        missing += bad ? 1 : 0;
        if (from_centre_m <= o.under_radius_m)
        {
            ++samples_under;
            missing_under += bad ? 1 : 0;
        }
    };
    // A 7 x 7 grid over the circle's square, a quarter of the radius apart.
    for (int i = -3; i <= 3; ++i)
    {
        for (int j = -3; j <= 3; ++j)
        {
            const double fx = 0.25 * i;
            const double fy = 0.25 * j;
            const double from_centre = std::hypot(fx, fy);
            if (from_centre > 1.0)
            {
                continue;
            }
            const double from_centre_m = from_centre * o.free_radius_m;
            const Eigen::Vector3d q =
                camera_from_base
                * (centre + (o.free_radius_m * ((area.lateral * fx) + (area.forward * fy))));
            if (q.z() <= 0.0)
            {
                count(true, from_centre_m);
                continue;
            }
            const double u = (camera.fx * q.x() / q.z()) + camera.cx;
            const double v = (camera.fy * q.y() / q.z()) + camera.cy;
            if (u < 0.0 || v < 0.0 || u >= depth_m.cols || v >= depth_m.rows)
            {
                count(true, from_centre_m);
                continue;
            }
            const double z = depth_m.at<float>(static_cast<int>(v), static_cast<int>(u));
            if (!std::isfinite(z) || z < o.min_depth_m || z > o.max_depth_m)
            {
                count(true, from_centre_m);
                continue;
            }
            const bool off_plane = std::abs(z - q.z()) > o.surface_tolerance_m;
            const Eigen::Vector3d seen =
                base_to_camera
                * Eigen::Vector3d(
                    (u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z);
            count(off_plane && !isHeld(seen, o), from_centre_m);
        }
    }
    return samples > 0 && static_cast<double>(missing) / samples <= o.max_no_depth_share
           && (samples_under == 0
               || static_cast<double>(missing_under) / samples_under <= o.max_no_depth_share_under);
}

// Whether a spot's whole free circle of radius_m lies on the top.
bool onTop(const TopRegion & top, const Eigen::Vector3d & spot, double radius_m)
{
    const Eigen::Vector3d from_centre = spot - top.center;
    return std::abs(from_centre.dot(top.lateral)) <= top.half_width_m - radius_m + 1e-9
           && std::abs(from_centre.dot(top.forward)) <= top.half_depth_m - radius_m + 1e-9;
}

// The depth image's points that stand on the top, base frame: higher than occupied_above_m, lower
// than an object can be, and farther from the camera than the gripper and the held object.
std::vector<Eigen::Vector3d> pointsOnTop(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const TopRegion & top,
    const FreeSpotOptions & options)
{
    std::vector<Eigen::Vector3d> points;
    const int stride = std::max(1, options.stride_px);
    const Eigen::Vector3d camera_position = base_to_camera.translation();
    for (int vpx = 0; vpx < depth_m.rows; vpx += stride)
    {
        const auto * row = depth_m.ptr<float>(vpx);
        for (int upx = 0; upx < depth_m.cols; upx += stride)
        {
            const float z = row[upx];
            if (!std::isfinite(z) || z < options.min_depth_m || z > options.max_depth_m)
            {
                continue;
            }
            const Eigen::Vector3d p =
                base_to_camera
                * Eigen::Vector3d(
                    (upx - camera.cx) * z / camera.fx, (vpx - camera.cy) * z / camera.fy, z);
            const double above_m = p.z() - top.center.z();
            // The held object and the gripper can be in the image too: nearer the camera than a
            // view stands, or higher above the top than any object.
            if (above_m > options.occupied_above_m && above_m < options.max_object_height_m
                && (p - camera_position).norm() > options.min_camera_distance_m
                && !isHeld(p, options))
            {
                points.push_back(p);
            }
        }
    }
    return points;
}

}  // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity): one grid search with its early-outs.
FreeSpotResult freeSpot(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const TopRegion & top,
    const TopRegion & area,
    const ViewConfig & view,
    const FreeSpotOptions & options)
{
    FreeSpotResult result;
    if (depth_m.empty() || depth_m.type() != CV_32FC1 || camera.fx <= 0.0 || camera.fy <= 0.0)
    {
        return result;
    }
    result.occupied = pointsOnTop(depth_m, camera, base_to_camera, top, options);
    const std::vector<Eigen::Vector3d> & occupied = result.occupied;
    FreeSpotCounts & counts = result.counts;
    static constexpr std::array<std::array<double, 2>, 8> kCircle{
        {{1.0, 0.0},
         {0.707, 0.707},
         {0.0, 1.0},
         {-0.707, 0.707},
         {-1.0, 0.0},
         {-0.707, -0.707},
         {0.0, -1.0},
         {0.707, -0.707}}};
    const double r = options.free_radius_m;
    const double seen_radius_m = std::max(r, options.tall_radius_m);
    // The grid is centred on the area's centre, so a place target is itself a candidate.
    const int across =
        static_cast<int>(std::floor(((area.half_width_m - r) / options.grid_step_m) + 1e-9));
    const int along =
        static_cast<int>(std::floor(((area.half_depth_m - r) / options.grid_step_m) + 1e-9));
    const Eigen::Isometry3d camera_from_base = base_to_camera.inverse();
    double best = std::numeric_limits<double>::max();
    for (int i = -across; i <= across; ++i)
    {
        const double a = i * options.grid_step_m;
        for (int j = -along; j <= along; ++j)
        {
            const double b = j * options.grid_step_m;
            const Eigen::Vector3d p = area.center + (area.lateral * a) + (area.forward * b);
            if (!onTop(top, p, r))
            {
                continue;
            }
            if (options.max_reach_m > 0.0 && p.head<2>().norm() > options.max_reach_m)
            {
                ++counts.out_of_reach;
                continue;
            }
            const double cost = std::hypot(a, b) + (options.near_weight * (b + area.half_depth_m));
            if (cost >= best)
            {
                continue;
            }
            ++counts.checked;
            // What the image does not show (out of view, behind the gripper) is unknown, not free:
            // the whole circle around a spot must be in view.
            if (!std::ranges::all_of(kCircle, [&](const auto & d) {
                    return inView(
                        base_to_camera,
                        p + (seen_radius_m * ((area.lateral * d[0]) + (area.forward * d[1]))),
                        view);
                }))
            {
                ++counts.unseen;
                continue;
            }
            int in_the_way = 0;
            for (const Eigen::Vector3d & q : occupied)
            {
                const double d = (q - p).head<2>().norm();
                const double above_m = q.z() - p.z();
                if (d < r || (d < options.tall_radius_m && above_m > options.tall_above_m)
                    || (d < options.wrist_radius_m && above_m > options.wrist_above_m))
                {
                    if (++in_the_way >= options.min_occupied_points)
                    {
                        break;
                    }
                }
            }
            if (in_the_way >= options.min_occupied_points)
            {
                ++counts.taken;
                continue;
            }
            if (!topInView(depth_m, camera, base_to_camera, camera_from_base, area, p, options))
            {
                ++counts.no_depth;
                continue;
            }
            best = cost;
            result.spot = p;
        }
    }
    return result;
}

std::optional<double> topHeightInView(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    const Eigen::Isometry3d & base_to_camera,
    const TopRegion & top,
    double search_m)
{
    if (depth_m.empty() || depth_m.type() != CV_32FC1 || camera.fx <= 0.0 || camera.fy <= 0.0
        || search_m <= 0.0)
    {
        return std::nullopt;
    }
    // 2 mm bins; every 8th pixel is plenty for a table filling part of the image.
    constexpr double kBinM = 0.002;
    constexpr int kStride = 8;
    constexpr int kMinPoints = 50;
    const auto bins = static_cast<std::size_t>(std::ceil(2.0 * search_m / kBinM)) + 1;
    std::vector<int> histogram(bins, 0);
    std::vector<double> sums(bins, 0.0);
    int on_the_rectangle = 0;
    for (int vpx = 0; vpx < depth_m.rows; vpx += kStride)
    {
        const auto * row = depth_m.ptr<float>(vpx);
        for (int upx = 0; upx < depth_m.cols; upx += kStride)
        {
            const double z = row[upx];
            if (!std::isfinite(z) || z <= 0.0)
            {
                continue;
            }
            const Eigen::Vector3d p =
                base_to_camera
                * Eigen::Vector3d(
                    (upx - camera.cx) * z / camera.fx, (vpx - camera.cy) * z / camera.fy, z);
            const Eigen::Vector3d d = p - top.center;
            if (std::abs(d.dot(top.lateral)) > top.half_width_m
                || std::abs(d.dot(top.forward)) > top.half_depth_m)
            {
                continue;
            }
            ++on_the_rectangle;
            if (std::abs(d.z()) > search_m)
            {
                continue;
            }
            const auto bin = static_cast<std::size_t>((d.z() + search_m) / kBinM);
            ++histogram[bin];
            sums[bin] += p.z();
        }
    }
    const auto peak = std::ranges::max_element(histogram);
    const auto peak_at = static_cast<std::size_t>(std::distance(histogram.begin(), peak));
    // Most of what the image shows of the rectangle should be the top itself.
    if (*peak < kMinPoints || 4 * *peak < on_the_rectangle)
    {
        return std::nullopt;
    }
    return sums[peak_at] / *peak;
}

Eigen::Vector3d boxCentre(
    const Eigen::Isometry3d & pose, const std::vector<Eigen::Vector3d> & points)
{
    if (points.empty())
    {
        return pose.translation();
    }
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d hi = -lo;
    for (const Eigen::Vector3d & q : points)
    {
        const Eigen::Vector3d local = pose.linear().transpose() * (q - pose.translation());
        lo = lo.cwiseMin(local);
        hi = hi.cwiseMax(local);
    }
    return pose.translation() + (pose.linear() * (0.5 * (lo + hi)));
}

Footprint footprintOf(const std::vector<Eigen::Vector3d> & points)
{
    Footprint f;
    if (points.empty())
    {
        return f;
    }
    const ObjectShape shape = describe(points);
    f.center = shape.centroid.head<2>();
    f.long_axis = shape.axes.col(0).head<2>().normalized();
    f.length_m = shape.size.x();
    f.breadth_m = shape.size.y();
    for (const Eigen::Vector3d & q : points)
    {
        f.top_z_m = std::max(f.top_z_m, q.z());
        f.radius_m = std::max(f.radius_m, (q.head<2>() - f.center).norm());
    }
    return f;
}

Eigen::Isometry3d placeTcp(
    const Eigen::Vector3d & spot,
    double turn_rad,
    const Eigen::Matrix3d & tcp_rotation,
    const Eigen::Vector3d & centre_to_tcp,
    double tcp_above_bottom_m,
    double release_height_m)
{
    const Eigen::Matrix3d rz =
        Eigen::AngleAxisd(turn_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
    tcp.linear() = rz * tcp_rotation;
    const Eigen::Vector3d offset = rz * centre_to_tcp;
    tcp.translation() = Eigen::Vector3d(
        spot.x() + offset.x(),
        spot.y() + offset.y(),
        spot.z() + tcp_above_bottom_m + release_height_m);
    return tcp;
}

std::optional<TopRegion> placeArea(const TopRegion & top, const std::string & region)
{
    if (region == "any")
    {
        return top;
    }
    double shift = 0.0;
    if (region == "left")
    {
        shift = 2.0 / 3.0;
    }
    else if (region == "right")
    {
        shift = -2.0 / 3.0;
    }
    else if (region != "middle")
    {
        return std::nullopt;
    }
    TopRegion third = top;
    third.half_width_m = top.half_width_m / 3.0;
    third.center = top.center + (top.lateral * (shift * top.half_width_m));
    return third;
}

std::vector<std::string> viewOrder(
    const std::vector<TopView> & planned,
    const std::vector<std::string> & asked,
    const std::vector<std::string> & preferred)
{
    const auto exists = [&](const std::string & name) {
        return std::ranges::any_of(planned, [&](const TopView & v) { return v.name == name; });
    };
    std::vector<std::string> out;
    for (const std::string & name : asked.empty() ? preferred : asked)
    {
        if (exists(name) && std::ranges::find(out, name) == out.end())
        {
            out.push_back(name);
        }
    }
    if (asked.empty())
    {
        for (const TopView & v : planned)
        {
            if (std::ranges::find(out, v.name) == out.end())
            {
                out.push_back(v.name);
            }
        }
    }
    return out;
}

}  // namespace piper_skills
