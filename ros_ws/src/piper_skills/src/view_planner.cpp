#include "piper_skills/view_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace piper_skills
{
namespace
{

// Optical frame looking at the region's centre, tilted from straight down towards `forward`.
Eigen::Matrix3d orientation(const TopRegion & region, double tilt_rad)
{
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d z = -std::cos(tilt_rad) * up + std::sin(tilt_rad) * region.forward;
    // The image's top points away from the robot, so its right is the robot's right.
    const Eigen::Vector3d y = -(std::cos(tilt_rad) * region.forward + std::sin(tilt_rad) * up);
    Eigen::Matrix3d r;
    r.col(0) = y.cross(z);
    r.col(1) = y;
    r.col(2) = z;
    return r;
}

std::array<Eigen::Vector3d, 4> corners(const TopRegion & region)
{
    const Eigen::Vector3d a = region.lateral * region.half_width_m;
    const Eigen::Vector3d b = region.forward * region.half_depth_m;
    return {
        region.center + a + b, region.center + a - b, region.center - a + b, region.center - a - b};
}

bool fits(const TopRegion & region, const Eigen::Isometry3d & camera, const ViewConfig & config)
{
    return std::ranges::all_of(
        corners(region), [&](const Eigen::Vector3d & c) { return inView(camera, c, config); });
}

}  // namespace

bool inView(
    const Eigen::Isometry3d & camera, const Eigen::Vector3d & point, const ViewConfig & config)
{
    const Eigen::Vector3d p = camera.inverse() * point;
    if (p.z() <= 1.0e-6)
    {
        return false;
    }
    const double keep = 1.0 - 2.0 * config.image_margin;
    const double v_tan = std::tan(config.vfov_rad / 2.0);
    // Image y grows downwards: the bottom band the gripper hides is cut off.
    const double v = p.y() / p.z();
    return std::abs(p.x() / p.z()) <= std::tan(config.hfov_rad / 2.0) * keep && v >= -v_tan * keep
           && v <= v_tan * (keep - 2.0 * config.occluded_bottom);
}

std::vector<ViewCandidate> viewRegion(const TopRegion & region, const ViewConfig & config)
{
    std::vector<ViewCandidate> out;
    for (const double tilt_rad : config.tilts_rad)
    {
        const Eigen::Matrix3d r = orientation(region, tilt_rad);
        const auto camera_at = [&](double distance) {
            Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
            camera.linear() = r;
            camera.translation() = region.center - distance * r.col(2);
            return camera;
        };
        if (!fits(region, camera_at(config.max_distance_m), config))
        {
            continue;
        }
        // The projection shrinks with distance, so the closest fitting one is a bisection away.
        double lo = config.min_distance_m;
        double hi = config.max_distance_m;
        if (!fits(region, camera_at(lo), config))
        {
            for (int i = 0; i < 30; ++i)
            {
                const double mid = 0.5 * (lo + hi);
                (fits(region, camera_at(mid), config) ? hi : lo) = mid;
            }
            lo = hi;
        }
        out.push_back({camera_at(lo), tilt_rad, lo});
    }
    return out;
}

std::vector<TopView> planTopViews(const TopRegion & whole, const ViewConfig & config)
{
    TopRegion top = whole;
    if (config.max_depth_m > 0.0 && config.max_depth_m < 2.0 * whole.half_depth_m)
    {
        // The band along the near edge.
        top.half_depth_m = 0.5 * config.max_depth_m;
        top.center = whole.center - whole.forward * (whole.half_depth_m - top.half_depth_m);
    }
    std::vector<TopView> views{{"full", top, viewRegion(top, config)}};
    const int n = std::clamp(
        static_cast<int>(std::ceil(2.0 * top.half_width_m / config.section_max_width_m)),
        1,
        config.max_sections);
    if (n < 2)
    {
        return views;
    }
    const double step = 2.0 * top.half_width_m / n;
    for (int k = 0; k < n; ++k)
    {
        // k = 0 is the robot's leftmost section.
        TopRegion section = top;
        section.center = top.center + top.lateral * (top.half_width_m - (k + 0.5) * step);
        section.half_width_m = 0.5 * step * (1.0 + config.section_overlap);
        std::string name = "section_" + std::to_string(k);
        if (n == 2)
        {
            name = k == 0 ? "left" : "right";
        }
        else if (n == 3)
        {
            static constexpr std::array<const char *, 3> kThree{"left", "middle", "right"};
            name = kThree.at(static_cast<std::size_t>(k));
        }
        views.push_back({name, section, viewRegion(section, config)});
    }
    return views;
}

}  // namespace piper_skills
