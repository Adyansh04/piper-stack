#include "piper_skills/grasp_selection.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace piper_skills
{

namespace
{

// The approach's cosine with straight down.
double uprightness(const Eigen::Isometry3d & tcp)
{
    return -tcp.linear().col(2).z();
}

}  // namespace

bool isSideGrasp(const Eigen::Isometry3d & tcp, const GraspFilter & filter)
{
    return uprightness(tcp) < std::cos(filter.top_max_angle_rad);
}

GraspRanking rankGrasps(
    const std::vector<GraspCandidate> & grasps,
    const GraspFilter & filter,
    double top_height_m,
    double object_top_m,
    GraspApproach approach)
{
    GraspRanking out;
    out.verdicts.assign(grasps.size(), GraspVerdict::kRanked);
    std::vector<std::pair<double, std::size_t>> ranked;
    const double min_cos = std::cos(filter.max_approach_angle_rad);
    const double max_closing_sin = std::sin(filter.max_closing_tilt_rad);
    const double object_height_m = object_top_m - top_height_m;
    // Fingertips just under the top pinch an edge the object tips out of when lifted.
    const double min_depth_m =
        std::min(filter.min_depth_m, filter.min_depth_share * object_height_m);
    const double mid_z_m = top_height_m + 0.5 * object_height_m;
    const double max_offset_m = filter.side_max_offset_share * object_height_m;
    for (std::size_t i = 0; i < grasps.size(); ++i)
    {
        const GraspCandidate & g = grasps[i];
        const double upright = uprightness(g.tcp);
        const bool side = isSideGrasp(g.tcp, filter);
        // A top grasp's fingertips reach down from the object's top; a side grasp's into its face.
        const double depth_m = side ? g.depth_m : object_top_m - g.tcp.translation().z();
        const double lowest_m = side ? lowestFingerZ(g.tcp, filter.pads, filter.finger_thickness_m)
                                     : g.tcp.translation().z();
        // The middle of the object between the pads: half the depth back from the fingertips.
        const double grip_z_m = g.tcp.translation().z() - 0.5 * depth_m * g.tcp.linear().col(2).z();
        std::size_t * rejected = nullptr;
        if (upright < min_cos)
        {
            rejected = &out.too_tilted;
            out.verdicts[i] = GraspVerdict::kTooTilted;
        }
        else if (
            (approach == GraspApproach::kTop && side)
            || (approach == GraspApproach::kSide && !side))
        {
            rejected = &out.other_approach;
            out.verdicts[i] = GraspVerdict::kOtherApproach;
        }
        else if (side && std::abs(g.tcp.linear().col(1).z()) > max_closing_sin)
        {
            rejected = &out.finger_under;
            out.verdicts[i] = GraspVerdict::kFingerUnder;
        }
        else if (g.object_width_m < filter.min_width_m)
        {
            rejected = &out.too_narrow;
            out.verdicts[i] = GraspVerdict::kTooNarrow;
        }
        else if (g.width_m > filter.max_width_m)
        {
            rejected = &out.too_wide;
            out.verdicts[i] = GraspVerdict::kTooWide;
        }
        else if (lowest_m < top_height_m + filter.top_clearance_m)
        {
            rejected = &out.too_low;
            out.verdicts[i] = GraspVerdict::kTooLow;
        }
        else if (depth_m < min_depth_m)
        {
            rejected = &out.too_shallow;
            out.verdicts[i] = GraspVerdict::kTooShallow;
        }
        else if (depth_m > filter.max_depth_m)
        {
            rejected = &out.too_deep;
            out.verdicts[i] = GraspVerdict::kTooDeep;
        }
        else if (side && std::abs(grip_z_m - mid_z_m) > max_offset_m)
        {
            rejected = &out.off_centre;
            out.verdicts[i] = GraspVerdict::kOffCentre;
        }
        if (rejected != nullptr)
        {
            ++*rejected;
            continue;
        }
        ranked.emplace_back(
            g.score + filter.upright_weight * upright
                + (side ? filter.side_depth_weight : filter.depth_weight) * depth_m
                - filter.width_weight * g.width_m - filter.contact_weight * g.contact_spread_m,
            i);
    }
    std::ranges::sort(ranked, [](const auto & a, const auto & b) { return a.first > b.first; });
    out.order.reserve(ranked.size());
    for (const auto & r : ranked)
    {
        out.order.push_back(r.second);
    }
    return out;
}

double contactSpread(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads)
{
    const int strips = std::max(pads.strips, 1);
    // Outermost distance from the centre plane per side and strip, -1 where no point fell.
    std::vector<double> outermost(2 * static_cast<std::size_t>(strips), -1.0);
    const Eigen::Isometry3d to_tcp = tcp.inverse();
    const double half_width_m = 0.5 * pads.width_m;
    for (const Eigen::Vector3d & p : points)
    {
        const Eigen::Vector3d q = to_tcp * p;
        if (std::abs(q.x()) > half_width_m || q.z() > 0.0 || q.z() < -pads.length_m
            || std::abs(q.y()) > 0.5 * pads.open_gap_m)
        {
            continue;
        }
        const int strip = std::clamp(
            static_cast<int>((q.x() + half_width_m) / pads.width_m * strips), 0, strips - 1);
        double & out = outermost[(q.y() < 0.0 ? 0 : strips) + strip];
        out = std::max(out, std::abs(q.y()));
    }
    double spread_m = 0.0;
    for (int side = 0; side < 2; ++side)
    {
        double low = std::numeric_limits<double>::infinity();
        double high = -low;
        int seen = 0;
        for (int k = 0; k < strips; ++k)
        {
            const double out = outermost[(side * strips) + k];
            if (out >= 0.0)
            {
                low = std::min(low, out);
                high = std::max(high, out);
                ++seen;
            }
        }
        if (seen >= 2)
        {
            spread_m = std::max(spread_m, high - low);
        }
    }
    return spread_m;
}

double closingOffset(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads)
{
    const Eigen::Isometry3d to_tcp = tcp.inverse();
    double low = std::numeric_limits<double>::infinity();
    double high = -low;
    for (const Eigen::Vector3d & p : points)
    {
        const Eigen::Vector3d q = to_tcp * p;
        if (q.z() < 0.0 && q.z() >= -pads.length_m && std::abs(q.x()) <= 0.5 * pads.width_m
            && std::abs(q.y()) <= 0.5 * pads.open_gap_m)
        {
            low = std::min(low, q.y());
            high = std::max(high, q.y());
        }
    }
    return low <= high ? 0.5 * (low + high) : 0.0;
}

double graspDepth(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads)
{
    const Eigen::Isometry3d to_tcp = tcp.inverse();
    double depth_m = 0.0;
    for (const Eigen::Vector3d & p : points)
    {
        const Eigen::Vector3d q = to_tcp * p;
        // Behind the fingertips (-z), within the pads' width and length, between the pads.
        if (q.z() < 0.0 && q.z() >= -pads.length_m && std::abs(q.x()) <= 0.5 * pads.width_m
            && std::abs(q.y()) <= 0.5 * pads.open_gap_m)
        {
            depth_m = std::max(depth_m, -q.z());
        }
    }
    return depth_m;
}

double lowestFingerZ(
    const Eigen::Isometry3d & tcp, const PadGeometry & pads, double finger_thickness_m)
{
    // The fingers' box in the TCP frame: across the pads' width (x), out to the fingers' outer
    // faces (y), from the fingertips back along the pads (z); its lowest corner.
    const Eigen::Vector3d lo(
        -0.5 * pads.width_m, -(0.5 * pads.open_gap_m) - finger_thickness_m, -pads.length_m);
    const Eigen::Vector3d hi(0.5 * pads.width_m, (0.5 * pads.open_gap_m) + finger_thickness_m, 0.0);
    double z_m = tcp.translation().z();
    for (int k = 0; k < 3; ++k)
    {
        const double up = tcp.linear()(2, k);
        z_m += std::min(up * lo[k], up * hi[k]);
    }
    return z_m;
}

Eigen::Isometry3d backOff(const Eigen::Isometry3d & tcp, double distance_m)
{
    Eigen::Isometry3d out = tcp;
    out.translation() -= tcp.linear().col(2) * distance_m;
    return out;
}

}  // namespace piper_skills
