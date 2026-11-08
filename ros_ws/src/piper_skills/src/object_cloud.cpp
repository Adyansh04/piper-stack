#include "piper_skills/object_cloud.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <opencv2/imgproc.hpp>

namespace piper_skills
{

std::vector<Eigen::Vector3f> maskPoints(
    const cv::Mat & depth_m,
    const cv::Mat & mask,
    const CameraModel & camera,
    const CloudOptions & options)
{
    std::vector<Eigen::Vector3f> points;
    if (depth_m.empty() || mask.size() != depth_m.size() || depth_m.type() != CV_32FC1
        || camera.fx <= 0.0 || camera.fy <= 0.0)
    {
        return points;
    }
    cv::Mat inner;
    if (options.erode_px > 0)
    {
        const int k = 2 * options.erode_px + 1;
        cv::erode(mask, inner, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));
    }
    else
    {
        inner = mask;
    }
    std::vector<float> depths;
    std::vector<cv::Point> pixels;
    for (int v = 0; v < inner.rows; ++v)
    {
        const auto * m = inner.ptr<unsigned char>(v);
        const auto * d = depth_m.ptr<float>(v);
        for (int u = 0; u < inner.cols; ++u)
        {
            if (m[u] != 0 && std::isfinite(d[u]) && d[u] >= options.min_depth_m
                && d[u] <= options.max_depth_m)
            {
                depths.push_back(d[u]);
                pixels.emplace_back(u, v);
            }
        }
    }
    if (depths.empty())
    {
        return points;
    }
    std::vector<float> sorted = depths;
    std::nth_element(
        sorted.begin(),
        sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2),
        sorted.end());
    const float median = sorted[sorted.size() / 2];
    points.reserve(depths.size());
    for (std::size_t i = 0; i < depths.size(); ++i)
    {
        const float z = depths[i];
        if (std::abs(z - median) > options.depth_band_m)
        {
            continue;
        }
        points.emplace_back(
            static_cast<float>((pixels[i].x - camera.cx) * z / camera.fx),
            static_cast<float>((pixels[i].y - camera.cy) * z / camera.fy),
            z);
    }
    return points;
}

namespace
{

using Voxel = std::array<int, 3>;

// Every voxel reachable from `start` through touching occupied voxels gets cluster `id`; returns
// how many points the cluster holds.
std::size_t floodFill(
    const std::map<Voxel, std::vector<std::size_t>> & voxels,
    const Voxel & start,
    int id,
    std::map<Voxel, int> & cluster_of)
{
    std::size_t count = 0;
    std::vector<Voxel> stack{start};
    cluster_of[start] = id;
    while (!stack.empty())
    {
        const Voxel v = stack.back();
        stack.pop_back();
        count += voxels.at(v).size();
        for (int n = 0; n < 27; ++n)
        {
            const Voxel w{v[0] + (n % 3) - 1, v[1] + ((n / 3) % 3) - 1, v[2] + (n / 9) - 1};
            if (voxels.contains(w) && !cluster_of.contains(w))
            {
                cluster_of[w] = id;
                stack.push_back(w);
            }
        }
    }
    return count;
}

}  // namespace

std::vector<Eigen::Vector3d> ownPoints(
    const std::vector<Eigen::Vector3d> & points, double top_z_m, double above_top_m, double gap_m)
{
    std::vector<Eigen::Vector3d> above;
    above.reserve(points.size());
    for (const Eigen::Vector3d & p : points)
    {
        // A NaN top fails the comparison, so every point stays.
        if (!(p.z() <= top_z_m + above_top_m))
        {
            above.push_back(p);
        }
    }
    if (gap_m <= 0.0 || above.empty())
    {
        return above;
    }
    std::map<Voxel, std::vector<std::size_t>> voxels;
    std::vector<Voxel> voxel_of(above.size());
    for (std::size_t i = 0; i < above.size(); ++i)
    {
        voxel_of[i] = {
            static_cast<int>(std::floor(above[i].x() / gap_m)),
            static_cast<int>(std::floor(above[i].y() / gap_m)),
            static_cast<int>(std::floor(above[i].z() / gap_m))};
        voxels[voxel_of[i]].push_back(i);
    }
    std::map<Voxel, int> cluster_of;
    std::vector<std::size_t> sizes;
    for (const auto & entry : voxels)
    {
        if (!cluster_of.contains(entry.first))
        {
            sizes.push_back(
                floodFill(voxels, entry.first, static_cast<int>(sizes.size()), cluster_of));
        }
    }
    const auto largest = static_cast<int>(std::ranges::max_element(sizes) - sizes.begin());
    // The input's order, so the result does not depend on how the voxels sort.
    std::vector<Eigen::Vector3d> out;
    out.reserve(static_cast<std::size_t>(sizes[static_cast<std::size_t>(largest)]));
    for (std::size_t i = 0; i < above.size(); ++i)
    {
        if (cluster_of.at(voxel_of[i]) == largest)
        {
            out.push_back(above[i]);
        }
    }
    return out;
}

ObjectShape describe(const std::vector<Eigen::Vector3d> & points)
{
    ObjectShape shape;
    if (points.empty())
    {
        return shape;
    }
    for (const Eigen::Vector3d & p : points)
    {
        shape.centroid += p;
    }
    shape.centroid /= static_cast<double>(points.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const Eigen::Vector3d & p : points)
    {
        const Eigen::Vector2d d = (p - shape.centroid).head<2>();
        cov += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
    // Eigen sorts ascending: the largest spread is the last vector.
    const Eigen::Vector2d major = eig.eigenvectors().col(1);
    shape.axes.col(0) = Eigen::Vector3d(major.x(), major.y(), 0.0);
    shape.axes.col(2) = Eigen::Vector3d::UnitZ();
    shape.axes.col(1) = shape.axes.col(2).cross(shape.axes.col(0));
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1.0e9);
    Eigen::Vector3d hi = -lo;
    for (const Eigen::Vector3d & p : points)
    {
        const Eigen::Vector3d local = shape.axes.transpose() * (p - shape.centroid);
        lo = lo.cwiseMin(local);
        hi = hi.cwiseMax(local);
    }
    shape.size = hi - lo;
    return shape;
}

std::vector<Eigen::Vector3f> depthPoints(
    const cv::Mat & depth_m,
    const CameraModel & camera,
    int step_px,
    double min_depth_m,
    double max_depth_m)
{
    std::vector<Eigen::Vector3f> points;
    if (depth_m.empty() || depth_m.type() != CV_32FC1 || camera.fx <= 0.0 || camera.fy <= 0.0)
    {
        return points;
    }
    const int step = std::max(step_px, 1);
    points.reserve(
        static_cast<std::size_t>((depth_m.rows / step) + 1)
        * static_cast<std::size_t>((depth_m.cols / step) + 1));
    for (int v = 0; v < depth_m.rows; v += step)
    {
        const auto * d = depth_m.ptr<float>(v);
        for (int u = 0; u < depth_m.cols; u += step)
        {
            const float z = d[u];
            if (std::isfinite(z) && z >= min_depth_m && z <= max_depth_m)
            {
                points.emplace_back(
                    static_cast<float>((u - camera.cx) * z / camera.fx),
                    static_cast<float>((v - camera.cy) * z / camera.fy),
                    z);
            }
        }
    }
    return points;
}

void drawInstance(
    cv::Mat & image,
    const cv::Mat & mask,
    const cv::Rect & box,
    const cv::Scalar & colour,
    const std::string & label)
{
    const cv::Rect inside = box & cv::Rect(0, 0, image.cols, image.rows);
    if (inside.empty() || mask.size() != box.size() || mask.type() != CV_8UC1)
    {
        return;
    }
    cv::Mat region = image(inside);
    cv::Mat tinted;
    cv::addWeighted(region, 0.5, cv::Mat(region.size(), region.type(), colour), 0.5, 0.0, tinted);
    tinted.copyTo(region, mask(inside - box.tl()));
    cv::rectangle(image, inside, colour, 2);
    constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
    constexpr double kScale = 0.5;
    int baseline = 0;
    const cv::Size text = cv::getTextSize(label, kFont, kScale, 1, &baseline);
    const int y = inside.y > text.height + 6 ? inside.y - 5 : inside.y + text.height + 5;
    // On black, so the label reads on any background.
    cv::rectangle(
        image,
        cv::Point(inside.x, y - text.height - 3),
        cv::Point(inside.x + text.width + 4, y + baseline),
        cv::Scalar::all(0),
        cv::FILLED);
    cv::putText(image, label, cv::Point(inside.x + 2, y), kFont, kScale, colour, 1, cv::LINE_AA);
}

}  // namespace piper_skills
