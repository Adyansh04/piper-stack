#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "piper_skills/grasp_selection.hpp"
#include "piper_skills/object_cloud.hpp"
#include "piper_skills/place_geometry.hpp"
#include "piper_skills/srdf_states.hpp"
#include "piper_skills/view_planner.hpp"

namespace piper_skills
{
namespace
{

TopRegion tableTop()
{
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.half_width_m = 0.9;
    top.half_depth_m = 0.4;
    return top;
}

TEST(ViewPlanner, FullViewShowsTheWholeTopAndSectionsSplitIt)
{
    ViewConfig config;
    config.hfov_rad = 90.6 * std::numbers::pi / 180.0;
    config.vfov_rad = 74.3 * std::numbers::pi / 180.0;
    config.occluded_bottom = 0.25;
    config.max_depth_m = 0.5;
    const TopRegion top = tableTop();
    const auto views = planTopViews(top, config);
    ASSERT_EQ(views.size(), 4U);  // full, left, middle, right (1.8 m in 0.7 m sections)
    EXPECT_EQ(views[0].name, "full");
    EXPECT_EQ(views[1].name, "left");
    EXPECT_EQ(views[3].name, "right");
    EXPECT_GT(views[1].region.center.y(), 0.5);  // the robot's left
    ASSERT_FALSE(views[0].candidates.empty());
    for (const TopView & view : views)
    {
        for (const ViewCandidate & c : view.candidates)
        {
            for (const double a : {-1.0, 1.0})
            {
                for (const double b : {-1.0, 1.0})
                {
                    const Eigen::Vector3d corner =
                        view.region.center + a * view.region.lateral * view.region.half_width_m
                        + b * view.region.forward * view.region.half_depth_m;
                    EXPECT_TRUE(inView(c.camera, corner, config))
                        << view.name << " tilt " << c.tilt_rad;
                }
            }
            // It looks at the region's centre.
            const Eigen::Vector3d to_center =
                (view.region.center - c.camera.translation()).normalized();
            EXPECT_NEAR(to_center.dot(c.camera.linear().col(2)), 1.0, 1e-9);
        }
    }
    // Straight down first, the image's right is the robot's right.
    EXPECT_NEAR(views[0].candidates[0].tilt_rad, 0.0, 1e-12);
    EXPECT_NEAR(views[0].candidates[0].camera.linear().col(0).y(), -1.0, 1e-9);
    // Sections are closer than the full view.
    EXPECT_LT(views[2].candidates[0].distance_m, views[0].candidates[0].distance_m);
}

TEST(ObjectCloud, DepthPointsSubsampleAndKeepTheRange)
{
    cv::Mat depth(4, 6, CV_32FC1, cv::Scalar(1.0F));
    depth.at<float>(0, 2) = 0.0F;  // no reading
    depth.at<float>(2, 2) = 3.0F;  // too far
    const CameraModel camera{.fx = 100.0, .fy = 100.0, .cx = 2.0, .cy = 2.0};
    const auto points = depthPoints(depth, camera, 2, 0.1, 2.0);
    // Pixels (0,0) (0,4) (2,0) (2,4) of the six sampled; (0,2) and (2,2) dropped.
    ASSERT_EQ(points.size(), 4U);
    EXPECT_NEAR(points[0].x(), -0.02, 1e-6);  // (0 - 2) * 1 m / 100 px
    EXPECT_NEAR(points[0].y(), -0.02, 1e-6);
    EXPECT_NEAR(points[0].z(), 1.0, 1e-6);
    EXPECT_TRUE(depthPoints(cv::Mat(), camera, 2, 0.1, 2.0).empty());
}

TEST(ObjectCloud, KeepsTheObjectAndDropsBackgroundThroughTheMask)
{
    cv::Mat depth(100, 100, CV_32FC1, cv::Scalar(1.0F));
    depth(cv::Rect(40, 40, 20, 20)).setTo(0.8F);
    depth.at<float>(50, 50) = 1.5F;  // a hole seen through to the floor
    const cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(0));
    mask(cv::Rect(38, 38, 24, 24)).setTo(255);  // a little larger than the object
    const CameraModel camera{.fx = 100.0, .fy = 100.0, .cx = 50.0, .cy = 50.0};
    CloudOptions options;
    options.erode_px = 3;
    options.depth_band_m = 0.1;
    const auto points = maskPoints(depth, mask, camera, options);
    ASSERT_FALSE(points.empty());
    for (const Eigen::Vector3f & p : points)
    {
        EXPECT_NEAR(p.z(), 0.8F, 1e-6F);
    }
}

TEST(ObjectCloud, DescribesARotatedBlock)
{
    std::vector<Eigen::Vector3d> points;
    const double yaw = 0.5;
    const Eigen::Rotation2Dd r(yaw);
    // A 20 x 10 cm grid every 1 cm.
    for (int i = -10; i <= 10; ++i)
    {
        for (int j = -5; j <= 5; ++j)
        {
            const Eigen::Vector2d p = r * Eigen::Vector2d(0.01 * i, 0.01 * j);
            points.emplace_back(1.0 + p.x(), 0.2 + p.y(), 0.7);
        }
    }
    const ObjectShape shape = describe(points);
    EXPECT_NEAR(shape.centroid.x(), 1.0, 1e-9);
    EXPECT_NEAR(
        std::abs(shape.axes.col(0).dot(Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0))),
        1.0,
        1e-6);
    EXPECT_NEAR(shape.size.x(), 0.2, 1e-6);
    EXPECT_NEAR(shape.size.y(), 0.1, 1e-6);
}

TEST(ObjectCloud, OwnPointsDropTheTopAndANeighbour)
{
    // A 10 x 6 x 10 cm block every 1 cm on a top at 0.65 m, a 30 cm strip of the top beside it (the
    // mask spilling over), and a small part 8 cm away.
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i <= 10; ++i)
    {
        for (int j = 0; j <= 6; ++j)
        {
            for (int k = 0; k <= 10; ++k)
            {
                points.emplace_back(1.0 + 0.01 * i, 0.01 * j, 0.65 + 0.01 * k);
            }
        }
    }
    const std::size_t block = points.size();
    for (int i = 0; i <= 30; ++i)
    {
        points.emplace_back(1.0 + 0.01 * i, -0.02, 0.651);
    }
    for (int k = 0; k <= 3; ++k)
    {
        points.emplace_back(1.18, 0.05, 0.66 + 0.01 * k);
    }
    // The block's bottom layer is in the slice above the top, as is the strip.
    const auto own = ownPoints(points, 0.65, 0.006, 0.02);
    EXPECT_EQ(own.size(), block - (std::size_t{11} * 7));
    EXPECT_NEAR(describe(own).size.x(), 0.1, 1e-9);
    // Without a top or a gap nothing is dropped; NaN keeps every height.
    EXPECT_EQ(ownPoints(points, std::nan(""), 0.006, 0.0).size(), points.size());
}

TEST(ObjectCloud, DrawInstanceTintsOnlyTheMaskAndClipsTheBox)
{
    cv::Mat image(100, 100, CV_8UC3, cv::Scalar::all(0));
    const cv::Mat mask(40, 40, CV_8UC1, cv::Scalar(0));
    mask(cv::Rect(10, 10, 20, 20)).setTo(255);
    drawInstance(image, mask, cv::Rect(30, 40, 40, 40), cv::Scalar(200, 0, 0), "cube 0.90");
    EXPECT_EQ(image.at<cv::Vec3b>(60, 50)[0], 100);  // in the mask: half the colour
    EXPECT_EQ(image.at<cv::Vec3b>(45, 45)[0], 0);    // in the box, outside the mask
    EXPECT_EQ(image.at<cv::Vec3b>(60, 30)[0], 200);  // the outline
    EXPECT_EQ(image.at<cv::Vec3b>(90, 10)[0], 0);
    // A box reaching past the image's corner: the part inside is drawn.
    drawInstance(image, mask, cv::Rect(80, 80, 40, 40), cv::Scalar(0, 200, 0), "edge");
    EXPECT_EQ(image.at<cv::Vec3b>(95, 95)[1], 100);
}

// A camera 1 m straight above a flat top at 0.65 m: optical z down, x along the base's x.
Eigen::Isometry3d cameraAbove(const Eigen::Vector3d & over)
{
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
    t.translation() = over + Eigen::Vector3d(0.0, 0.0, 1.0);
    return t;
}

TEST(PlaceGeometry, FreeSpotKeepsClearOfAnObjectAndOfMissingDepth)
{
    const CameraModel camera{.fx = 320.0, .fy = 320.0, .cx = 320.0, .cy = 240.0};
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.lateral = Eigen::Vector3d::UnitY();
    top.forward = Eigen::Vector3d::UnitX();
    top.half_width_m = 0.35;
    top.half_depth_m = 0.25;
    const Eigen::Isometry3d base_to_camera = cameraAbove(top.center);
    ViewConfig view;
    view.hfov_rad = 2.0 * std::atan(camera.cx / camera.fx);  // the test camera's fields of view
    view.vfov_rad = 2.0 * std::atan(camera.cy / camera.fy);
    view.occluded_bottom = 0.0;
    FreeSpotOptions options;
    options.free_radius_m = 0.08;
    // A const header: its pixels are written through regions of it below.
    const cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(1.0F));
    const FreeSpotResult empty = freeSpot(depth, camera, base_to_camera, top, top, view, options);
    ASSERT_TRUE(empty.spot.has_value());
    EXPECT_TRUE(empty.occupied.empty());
    // A 10 cm tall object over a patch of the image, near where the empty search chose.
    const cv::Rect patch(290, 210, 60, 60);
    depth(patch).setTo(0.9F);
    const FreeSpotResult beside = freeSpot(depth, camera, base_to_camera, top, top, view, options);
    ASSERT_TRUE(beside.spot.has_value());
    EXPECT_FALSE(beside.occupied.empty());
    const Eigen::Vector3d at = beside.spot.value_or(Eigen::Vector3d::Constant(std::nan("")));
    for (const Eigen::Vector3d & q : beside.occupied)
    {
        EXPECT_GE((q - at).head<2>().norm(), options.free_radius_m - 1e-9);
    }
    // Around the object only: every candidate is taken, and counted so.
    TopRegion around = top;
    around.half_width_m = options.free_radius_m;
    around.half_depth_m = options.free_radius_m;
    const FreeSpotResult taken =
        freeSpot(depth, camera, base_to_camera, top, around, view, options);
    EXPECT_FALSE(taken.spot.has_value());
    EXPECT_EQ(taken.counts.checked, 1);
    EXPECT_EQ(taken.counts.taken, 1);
    // Without depth anywhere nothing counts as free table.
    const cv::Mat none(480, 640, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    const FreeSpotResult blind = freeSpot(none, camera, base_to_camera, top, top, view, options);
    EXPECT_FALSE(blind.spot.has_value());
    EXPECT_EQ(blind.counts.no_depth, blind.counts.checked);
    EXPECT_GT(blind.counts.checked, 0);
    // A board 0.45 m over the top (a shelf): the held object and the gripper would meet it coming
    // down, so the spot under it is taken too.
    const cv::Mat shelf(480, 640, CV_32FC1, cv::Scalar(1.0F));
    shelf(patch).setTo(0.55F);
    const FreeSpotResult under =
        freeSpot(shelf, camera, base_to_camera, top, around, view, options);
    EXPECT_FALSE(under.spot.has_value());
    EXPECT_EQ(under.counts.taken, 1);
}

TEST(PlaceGeometry, FreeSpotStaysWithinTheArmsReach)
{
    const CameraModel camera{.fx = 320.0, .fy = 320.0, .cx = 320.0, .cy = 240.0};
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.half_width_m = 0.35;
    top.half_depth_m = 0.25;
    ViewConfig view;
    view.hfov_rad = 2.0 * std::atan(camera.cx / camera.fx);
    view.vfov_rad = 2.0 * std::atan(camera.cy / camera.fy);
    view.occluded_bottom = 0.0;
    FreeSpotOptions options;
    options.free_radius_m = 0.08;
    options.max_reach_m = 0.9;
    const cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(1.0F));
    const Eigen::Isometry3d base_to_camera = cameraAbove(top.center);
    // The top's centre is 1 m from the base: the spot moves in to the reach.
    const FreeSpotResult near = freeSpot(depth, camera, base_to_camera, top, top, view, options);
    ASSERT_TRUE(near.spot.has_value());
    EXPECT_LE(near.spot.value_or(Eigen::Vector3d::Zero()).head<2>().norm(), 0.9 + 1e-9);
    EXPECT_GT(near.counts.out_of_reach, 0);
    // The whole top beyond it: none, and nothing looked at.
    options.max_reach_m = 0.5;
    const FreeSpotResult none = freeSpot(depth, camera, base_to_camera, top, top, view, options);
    EXPECT_FALSE(none.spot.has_value());
    EXPECT_EQ(none.counts.checked, 0);
    EXPECT_GT(none.counts.out_of_reach, 0);
}

TEST(PlaceGeometry, FreeSpotKeepsTheWristClearOfWhatStandsHighNearIt)
{
    const CameraModel camera{.fx = 320.0, .fy = 320.0, .cx = 320.0, .cy = 240.0};
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.lateral = Eigen::Vector3d::UnitY();
    top.forward = Eigen::Vector3d::UnitX();
    top.half_width_m = 0.35;
    top.half_depth_m = 0.25;
    const Eigen::Isometry3d base_to_camera = cameraAbove(top.center);
    ViewConfig view;
    view.hfov_rad = 2.0 * std::atan(camera.cx / camera.fx);  // the test camera's fields of view
    view.vfov_rad = 2.0 * std::atan(camera.cy / camera.fy);
    view.occluded_bottom = 0.0;
    FreeSpotOptions options;
    options.free_radius_m = 0.08;
    // The top's centre is the only candidate.
    TopRegion centre = top;
    centre.half_width_m = options.free_radius_m;
    centre.half_depth_m = options.free_radius_m;
    // Something 5 cm high and 17 cm from the centre: clear of the fingers, so the spot is free.
    const cv::Mat low(480, 640, CV_32FC1, cv::Scalar(1.0F));
    low(cv::Rect(375, 238, 4, 4)).setTo(0.95F);  // (u - cx) x depth / fx = 0.17 m
    EXPECT_TRUE(freeSpot(low, camera, base_to_camera, top, centre, view, options).spot.has_value());
    // A bar 0.3 m high at the same distance is where the wrist's camera comes down.
    const cv::Mat bar(480, 640, CV_32FC1, cv::Scalar(1.0F));
    bar(cv::Rect(398, 238, 4, 4)).setTo(0.7F);  // 78 px x 0.7 m / 320 px = 0.17 m
    const FreeSpotResult high = freeSpot(bar, camera, base_to_camera, top, centre, view, options);
    EXPECT_FALSE(high.spot.has_value());
    EXPECT_EQ(high.counts.taken, 1);
}

TEST(PlaceGeometry, ATargetsSmallCircleLetsLowObjectsCloseAndKeepsTallOnesAway)
{
    const CameraModel camera{.fx = 320.0, .fy = 320.0, .cx = 320.0, .cy = 240.0};
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.lateral = Eigen::Vector3d::UnitY();
    top.forward = Eigen::Vector3d::UnitX();
    top.half_width_m = 0.35;
    top.half_depth_m = 0.25;
    const Eigen::Isometry3d base_to_camera = cameraAbove(top.center);
    ViewConfig view;
    view.hfov_rad = 2.0 * std::atan(camera.cx / camera.fx);  // the test camera's fields of view
    view.vfov_rad = 2.0 * std::atan(camera.cy / camera.fy);
    view.occluded_bottom = 0.0;
    FreeSpotOptions options;
    options.free_radius_m = 0.07;
    options.tall_above_m = 0.12;
    options.tall_radius_m = 0.12;
    // The top's centre is the only candidate.
    TopRegion centre = top;
    centre.half_width_m = options.free_radius_m;
    centre.half_depth_m = options.free_radius_m;
    const auto spot_with = [&](int u_px, float depth_m) {
        const cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(1.0F));
        depth(cv::Rect(u_px, 238, 4, 4)).setTo(depth_m);
        return freeSpot(depth, camera, base_to_camera, top, centre, view, options).spot.has_value();
    };
    // 5 cm high, 9 cm from the centre (a neighbour's near face in a row): the fingers clear it.
    EXPECT_TRUE(spot_with(350, 0.95F));
    // 5 cm high, 5 cm from the centre: under the fingers.
    EXPECT_FALSE(spot_with(336, 0.95F));
    // 15 cm high, 9 cm from the centre: where the camera comes down.
    EXPECT_FALSE(spot_with(354, 0.85F));
    // A tall thing out of view may stand within the larger circle: all of it must be in view.
    TopRegion far_edge = centre;
    far_edge.center = top.center + Eigen::Vector3d(0.0, 0.0, 0.0);
    view.hfov_rad = 2.0 * std::atan(0.1);
    const cv::Mat empty(480, 640, CV_32FC1, cv::Scalar(1.0F));
    EXPECT_FALSE(
        freeSpot(empty, camera, base_to_camera, top, far_edge, view, options).spot.has_value());
}

TEST(PlaceGeometry, BoxCentreIsTheMiddleOfTheExtent)
{
    const Eigen::Isometry3d pose(Eigen::Translation3d(1.0, 0.0, 0.7));
    const std::vector<Eigen::Vector3d> points{
        {0.97, 0.0, 0.7}, {1.09, 0.0, 0.7}, {1.0, -0.06, 0.7}, {1.0, 0.06, 0.7}};
    const Eigen::Vector3d c = boxCentre(pose, points);
    EXPECT_NEAR(c.x(), 1.03, 1e-9);  // [-0.03, +0.09] about the centroid
    EXPECT_NEAR(c.y(), 0.0, 1e-9);
    EXPECT_NEAR(boxCentre(pose, {}).x(), 1.0, 1e-12);
}

TEST(PlaceGeometry, FootprintOfABlockAndOfNothing)
{
    EXPECT_DOUBLE_EQ(footprintOf({}).radius_m, 0.0);
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i <= 20; ++i)
    {
        for (int j = 0; j <= 10; ++j)
        {
            points.emplace_back(0.01 * i, 0.01 * j, 0.7 + (0.001 * j));
        }
    }
    const Footprint f = footprintOf(points);
    EXPECT_NEAR(f.length_m, 0.2, 1e-6);
    EXPECT_NEAR(f.breadth_m, 0.1, 1e-6);
    EXPECT_NEAR(f.top_z_m, 0.71, 1e-9);
    EXPECT_NEAR(std::abs(f.long_axis.x()), 1.0, 1e-6);
}

TEST(PlaceGeometry, PlaceTcpPutsTheCentreOnTheSpotTurned)
{
    const Eigen::Vector3d spot(1.0, 0.2, 0.65);
    const Eigen::Isometry3d tcp = placeTcp(
        spot,
        std::numbers::pi / 2.0,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.02, 0.0, 0.1),
        0.06,
        0.01);
    EXPECT_NEAR(tcp.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(tcp.translation().y(), 0.22, 1e-9);  // the offset turned with the object
    EXPECT_NEAR(tcp.translation().z(), 0.72, 1e-9);
    EXPECT_NEAR(tcp.linear()(1, 0), 1.0, 1e-9);
}

TEST(PlaceGeometry, PlaceAreasAreThirdsAndUnknownNamesAreRefused)
{
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.lateral = Eigen::Vector3d::UnitY();
    top.half_width_m = 0.9;
    // value_or's default region fails every check below.
    const TopRegion left = placeArea(top, "left").value_or(TopRegion{});
    EXPECT_NEAR(left.center.y(), 0.6, 1e-9);
    EXPECT_NEAR(left.half_width_m, 0.3, 1e-9);
    EXPECT_NEAR(placeArea(top, "right").value_or(TopRegion{}).center.y(), -0.6, 1e-9);
    EXPECT_NEAR(placeArea(top, "any").value_or(TopRegion{}).half_width_m, 0.9, 1e-12);
    EXPECT_FALSE(placeArea(top, "top").has_value());
}

TEST(PlaceGeometry, ViewOrderKeepsWhatExists)
{
    std::vector<TopView> planned(4);
    planned[0].name = "full";
    planned[1].name = "left";
    planned[2].name = "middle";
    planned[3].name = "right";
    EXPECT_EQ(
        viewOrder(planned, {}, {"middle", "section_4", "full"}),
        (std::vector<std::string>{"middle", "full", "left", "right"}));
    EXPECT_EQ(
        viewOrder(planned, {"right", "bogus"}, {"full"}), (std::vector<std::string>{"right"}));
}

GraspCandidate graspAt(double z_m, const Eigen::Vector3d & approach, double score, double width_m)
{
    GraspCandidate g;
    const Eigen::Vector3d zc = approach.normalized();
    const Eigen::Vector3d yc = zc.unitOrthogonal();
    g.tcp.linear().col(0) = yc.cross(zc);
    g.tcp.linear().col(1) = yc;
    g.tcp.linear().col(2) = zc;
    g.tcp.translation() = Eigen::Vector3d(1.0, 0.0, z_m);
    g.score = score;
    g.width_m = width_m;
    g.object_width_m = width_m;
    return g;
}

TEST(GraspSelection, KeepsGraspsTheGripperAndTableAllowBestFirst)
{
    const Eigen::Vector3d down(0.0, 0.0, -1.0);
    // A 5 cm wide, 8 cm high object on a top at 0.65 m: its top at 0.73 m.
    const std::vector<GraspCandidate> grasps{
        graspAt(0.70, down, 0.6, 0.05),                             // fine, 3 cm deep
        graspAt(0.69, Eigen::Vector3d(1.0, 0.0, 0.0), 0.9, 0.05),   // sideways
        graspAt(0.69, down, 0.9, 0.005),                            // narrower than min_width_m
        graspAt(0.655, down, 0.9, 0.05),                            // fingertips at the top
        graspAt(0.69, Eigen::Vector3d(0.5, 0.0, -1.0), 0.8, 0.05),  // tilted, more confident
        graspAt(0.72, down, 0.95, 0.05),                            // 1 cm deep: an edge pinch
        graspAt(0.68, down, 0.5, 0.05),                             // 5 cm deep
        graspAt(0.665, down, 0.9, 0.05),                            // 6.5 cm: the base jaws hit
        graspAt(0.69, Eigen::Vector3d(0.0, 0.0, 1.0), 0.9, 0.05),   // from below
    };
    GraspFilter filter;
    filter.width_weight = 0.0;
    const GraspRanking ranking = rankGrasps(grasps, filter, 0.65, 0.73, GraspApproach::kTop);
    const auto & order = ranking.order;
    ASSERT_EQ(order.size(), 3U);
    EXPECT_EQ(ranking.too_tilted, 1U);
    EXPECT_EQ(ranking.other_approach, 1U);  // the sideways one
    EXPECT_EQ(ranking.too_narrow, 1U);
    EXPECT_EQ(ranking.too_low, 1U);
    EXPECT_EQ(ranking.too_shallow, 1U);
    EXPECT_EQ(ranking.too_deep, 1U);
    // 0.8 + 0.5 x cos(26.6 deg) + 10 x 0.04 = 1.65; 0.5 + 0.5 + 0.5 = 1.5; 0.6 + 0.5 + 0.3 = 1.4.
    EXPECT_EQ(order[0], 4U);
    EXPECT_EQ(order[1], 6U);
    EXPECT_EQ(order[2], 0U);
    // Across a 4 cm box's faces (opening 4 cm) beats across its corners (5.7 cm) and the same box
    // 0.8 cm off centre (5.6 cm), at higher confidences.
    GraspCandidate off_centre = graspAt(0.69, down, 0.7, 0.056);
    off_centre.object_width_m = 0.04;
    const std::vector<GraspCandidate> box{
        graspAt(0.69, down, 0.7, 0.057), graspAt(0.69, down, 0.6, 0.04), off_centre};
    EXPECT_EQ(rankGrasps(box, GraspFilter{}, 0.65, 0.73).order.front(), 1U);
    const Eigen::Isometry3d pre = backOff(grasps[0].tcp, 0.1);
    EXPECT_NEAR(pre.translation().z(), 0.80, 1e-9);
}

// A grasp at `at` approaching along `approach`, closing along `closing` (both unit length).
GraspCandidate graspAlong(
    const Eigen::Vector3d & at, const Eigen::Vector3d & approach, const Eigen::Vector3d & closing)
{
    GraspCandidate g;
    g.tcp.linear().col(1) = closing;
    g.tcp.linear().col(2) = approach;
    g.tcp.linear().col(0) = closing.cross(approach);
    g.tcp.translation() = at;
    g.score = 0.5;
    g.width_m = 0.05;
    g.object_width_m = 0.05;
    g.depth_m = 0.03;
    return g;
}

TEST(GraspSelection, SideGraspsRankByKindAndKeepTheFingersAboveTheTop)
{
    // A 5 cm cube on a top at 0.65 m (its top at 0.70 m), grasped from the robot's side (+x).
    const Eigen::Vector3d ahead = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d across = Eigen::Vector3d::UnitY();
    // 63 degrees from straight down: flatter than a top grasp's 50.
    const Eigen::Vector3d diagonal = Eigen::Vector3d(1.0, 0.0, -0.5).normalized();
    const std::vector<GraspCandidate> grasps{
        graspAlong({0.995, 0.0, 0.675}, ahead, across),                    // side, fine
        graspAlong({0.995, 0.0, 0.675}, ahead, Eigen::Vector3d::UnitZ()),  // a finger under it
        graspAlong({0.995, 0.0, 0.665}, ahead, across),                    // fingers in the top
        graspAlong({0.995, 0.0, 0.678}, diagonal, across),                 // diagonal, fine
        graspAlong({1.0, 0.0, 0.67}, -Eigen::Vector3d::UnitZ(), across),   // top-down, fine
    };
    GraspFilter filter;
    filter.width_weight = 0.0;
    const GraspRanking side = rankGrasps(grasps, filter, 0.65, 0.70, GraspApproach::kSide);
    EXPECT_EQ(side.order, (std::vector<std::size_t>{3U, 0U}));  // the diagonal one is more upright
    EXPECT_EQ(side.finger_under, 1U);
    EXPECT_EQ(side.too_low, 1U);
    EXPECT_EQ(side.other_approach, 1U);
    EXPECT_EQ(side.verdicts[1], GraspVerdict::kFingerUnder);
    const GraspRanking top = rankGrasps(grasps, filter, 0.65, 0.70, GraspApproach::kTop);
    EXPECT_EQ(top.order, (std::vector<std::size_t>{4U}));
    EXPECT_EQ(top.other_approach, 4U);
    // Every kind together, depth weighed alike: the top-down grasp leads on uprightness. (A pick
    // ranks the top grasps and the side grasps in separate passes.)
    GraspFilter alike = filter;
    alike.side_depth_weight = alike.depth_weight;
    EXPECT_EQ(rankGrasps(grasps, alike, 0.65, 0.70).order.front(), 4U);
    EXPECT_TRUE(isSideGrasp(grasps[3].tcp, filter));
    EXPECT_FALSE(isSideGrasp(grasps[4].tcp, filter));
    // Too shallow and too deep apply to how far a side grasp reaches into the object.
    GraspCandidate shallow = grasps[0];
    shallow.depth_m = 0.01;
    GraspCandidate deep = grasps[0];
    deep.depth_m = 0.06;
    const GraspRanking depths =
        rankGrasps({shallow, deep}, filter, 0.65, 0.70, GraspApproach::kSide);
    EXPECT_EQ(depths.too_shallow, 1U);
    EXPECT_EQ(depths.too_deep, 1U);
    // A side grasp 4.5 cm into the cube outranks a narrower one 3 cm in: the deep one holds its
    // middle (with depth_weight's 10 per metre the narrower one would lead).
    const GraspCandidate & shallow_narrow = grasps[0];
    GraspCandidate deep_wide = grasps[0];
    deep_wide.depth_m = 0.045;
    deep_wide.width_m = 0.06;
    const GraspFilter ranked;  // the YAML's width weight
    EXPECT_EQ(
        rankGrasps({shallow_narrow, deep_wide}, ranked, 0.65, 0.70, GraspApproach::kSide).order,
        (std::vector<std::size_t>{1U, 0U}));
    // Gripping 2 cm above the cube's middle (0.675 m), 1.25 cm allowed: it turns out of the pads.
    GraspCandidate high = grasps[0];
    high.tcp.translation().z() = 0.695;
    const GraspRanking off = rankGrasps({high}, filter, 0.65, 0.70, GraspApproach::kSide);
    EXPECT_EQ(off.off_centre, 1U);
    EXPECT_EQ(off.verdicts[0], GraspVerdict::kOffCentre);
}

TEST(GraspSelection, GraspDepthAndTheLowestFinger)
{
    // A 5 cm cube's faces, sampled every 5 mm: x from 0.975 to 1.025 m, y +-0.025 m, z 0.65 to
    // 0.70 m.
    std::vector<Eigen::Vector3d> cube;
    for (int i = 0; i <= 10; ++i)
    {
        for (int j = 0; j <= 10; ++j)
        {
            const double a = 0.005 * i;
            const double b = 0.005 * j;
            cube.emplace_back(0.975, -0.025 + a, 0.65 + b);
            cube.emplace_back(1.025, -0.025 + a, 0.65 + b);
            cube.emplace_back(0.975 + a, -0.025, 0.65 + b);
            cube.emplace_back(0.975 + a, 0.025, 0.65 + b);
            cube.emplace_back(0.975 + a, -0.025 + b, 0.70);
        }
    }
    const PadGeometry pads;
    // Fingertips 2 cm past the front face: the cube reaches back 2 cm between the pads.
    const GraspCandidate side =
        graspAlong({0.995, 0.0, 0.675}, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
    EXPECT_NEAR(graspDepth(cube, side.tcp, pads), 0.02, 1e-9);  // metres
    // Past the pads' length nothing counts: 8.5 cm in, the pads reach back 6.5 cm.
    GraspCandidate far = side;
    far.tcp.translation().x() = 1.06;
    const double far_depth_m = graspDepth(cube, far.tcp, pads);
    EXPECT_LE(far_depth_m, pads.length_m);
    EXPECT_GT(far_depth_m, pads.length_m - 0.0051);  // within a sample spacing of the pads' length
    EXPECT_DOUBLE_EQ(graspDepth({}, side.tcp, pads), 0.0);
    // A horizontal approach with horizontal closing: the pads' width hangs half below the TCP.
    EXPECT_NEAR(lowestFingerZ(side.tcp, pads, 0.02), 0.675 - (0.5 * pads.width_m), 1e-12);
    // Straight down: the fingertips are the lowest point.
    const GraspCandidate down =
        graspAlong({1.0, 0.0, 0.68}, -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY());
    EXPECT_NEAR(lowestFingerZ(down.tcp, pads, 0.02), 0.68, 1e-12);
}

TEST(GraspSelection, ContactSpreadSeesASlantedEdge)
{
    // A 5.6 x 1.2 x 1 cm bar along x on a top at 0.65 m, sampled every 2 mm over its surface.
    constexpr double kStep = 0.002;
    std::vector<Eigen::Vector3d> bar;
    for (int i = -14; i <= 14; ++i)
    {
        const double x = kStep * i;
        for (int j = -3; j <= 3; ++j)
        {
            bar.emplace_back(x, kStep * j, 0.66);
        }
        for (int k = 0; k <= 5; ++k)
        {
            bar.emplace_back(x, -0.006, 0.65 + (kStep * k));
            bar.emplace_back(x, 0.006, 0.65 + (kStep * k));
        }
    }
    for (int j = -3; j <= 3; ++j)
    {
        for (int k = 0; k <= 5; ++k)
        {
            bar.emplace_back(-0.028, kStep * j, 0.65 + (kStep * k));
            bar.emplace_back(0.028, kStep * j, 0.65 + (kStep * k));
        }
    }
    // Top-down grasps with the fingertips 4 mm above the top, closing at yaw from the bar's length.
    const auto across = [](double yaw_rad) {
        GraspCandidate g;
        const Eigen::Vector3d closing(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
        g.tcp.linear().col(1) = closing;
        g.tcp.linear().col(2) = -Eigen::Vector3d::UnitZ();
        g.tcp.linear().col(0) = closing.cross(-Eigen::Vector3d::UnitZ());
        g.tcp.translation() = Eigen::Vector3d(0.0, 0.0, 0.654);
        return g;
    };
    const PadGeometry pads;
    // Along the length the pads meet the bar's square ends; 30 degrees off they meet its long edges.
    EXPECT_LT(contactSpread(bar, across(0.0).tcp, pads), 0.002);
    EXPECT_GT(contactSpread(bar, across(std::numbers::pi / 6.0).tcp, pads), 0.02);
    EXPECT_DOUBLE_EQ(contactSpread({}, across(0.0).tcp, pads), 0.0);

    // The slanted grasp needs the smaller opening, but the square one ranks first.
    GraspCandidate square = across(0.0);
    square.width_m = 0.056;
    square.object_width_m = 0.056;
    square.contact_spread_m = contactSpread(bar, square.tcp, pads);
    GraspCandidate slanted = across(std::numbers::pi / 6.0);
    slanted.width_m = 0.054;
    slanted.object_width_m = 0.054;
    slanted.contact_spread_m = contactSpread(bar, slanted.tcp, pads);
    GraspFilter filter;
    filter.top_clearance_m = 0.004;
    const GraspRanking ranking = rankGrasps({slanted, square}, filter, 0.65, 0.66);
    ASSERT_EQ(ranking.order.size(), 2U);
    EXPECT_EQ(ranking.order.front(), 1U);
}

TEST(PlaceGeometry, TheTopIsReadFromTheImageAndItsRealEdgeAndNoiseAreSeen)
{
    const CameraModel camera{.fx = 320.0, .fy = 320.0, .cx = 320.0, .cy = 240.0};
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.65);
    top.lateral = Eigen::Vector3d::UnitY();
    top.forward = Eigen::Vector3d::UnitX();
    top.half_width_m = 0.35;
    top.half_depth_m = 0.25;
    const Eigen::Isometry3d base_to_camera = cameraAbove(top.center);
    // A real top 1.5 cm above its model reads as such, with a box standing on it.
    const cv::Mat high(480, 640, CV_32FC1, cv::Scalar(0.985F));
    high(cv::Rect(300, 220, 40, 40)).setTo(0.9F);
    const auto seen = topHeightInView(high, camera, base_to_camera, top, 0.04);
    ASSERT_TRUE(seen.has_value());
    EXPECT_NEAR(*seen, 0.665, 0.002);  // NOLINT(bugprone-unchecked-optional-access): ASSERTed above
    EXPECT_FALSE(topHeightInView(
        cv::Mat(480, 640, CV_32FC1, cv::Scalar(0.5F)), camera, base_to_camera, top, 0.04));

    ViewConfig view;
    view.hfov_rad = 2.0 * std::atan(camera.cx / camera.fx);
    view.vfov_rad = 2.0 * std::atan(camera.cy / camera.fy);
    view.occluded_bottom = 0.0;
    FreeSpotOptions options;
    options.free_radius_m = 0.08;
    options.min_occupied_points = 5;
    TopRegion centre = top;
    centre.half_width_m = options.free_radius_m;
    centre.half_depth_m = options.free_radius_m;
    const auto free = [&](const cv::Mat & depth) {
        return freeSpot(depth, camera, base_to_camera, top, centre, view, options).spot.has_value();
    };
    const cv::Mat flat(480, 640, CV_32FC1, cv::Scalar(1.0F));
    EXPECT_TRUE(free(flat));
    // One bad pixel takes no spot; a patch does.
    cv::Mat speck = flat.clone();
    speck.at<float>(240, 330) = 0.9F;
    EXPECT_TRUE(free(speck));
    const cv::Mat patch = flat.clone();
    patch(cv::Rect(330, 236, 8, 8)).setTo(0.9F);
    EXPECT_FALSE(free(patch));
    // The floor past the real edge reads as valid depth, but it is no top.
    const cv::Mat edge = flat.clone();
    edge(cv::Rect(0, 0, 640, 240)).setTo(1.7F);
    EXPECT_FALSE(free(edge));
    // What returns no depth under the held object is not free ground.
    options.under_radius_m = 0.04;
    const cv::Mat black = flat.clone();
    black(cv::Rect(310, 230, 20, 20)).setTo(std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(free(black));
    options.under_radius_m = 0.0;
    // The held part in the image is neither an obstacle nor hides the top.
    const cv::Mat held = flat.clone();
    held(cv::Rect(330, 236, 8, 8)).setTo(0.6F);
    EXPECT_FALSE(free(held));
    options.tcp = Eigen::Vector3d(1.0, 0.0, 1.2);
    options.held_radius_m = 0.2;
    options.held_below_m = 0.25;
    EXPECT_TRUE(free(held));
}

TEST(GraspSelection, ClosingPushesTheObjectOntoTheFingersCentrePlane)
{
    const PadGeometry pads{.width_m = 0.0226, .length_m = 0.065, .open_gap_m = 0.0756};
    // Fingers 1 cm off a 5 cm cube's middle along the closing axis (base x), the pads down its sides.
    Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
    tcp.linear().col(0) = Eigen::Vector3d::UnitY();
    tcp.linear().col(1) = Eigen::Vector3d::UnitX();
    tcp.linear().col(2) = -Eigen::Vector3d::UnitZ();
    tcp.translation() = Eigen::Vector3d(0.71, 0.0, 0.66);
    std::vector<Eigen::Vector3d> cube;
    for (int i = 0; i <= 10; ++i)
    {
        cube.emplace_back(0.675 + 0.005 * i, 0.0, 0.70);
    }
    EXPECT_NEAR(closingOffset(cube, tcp, pads), -0.01, 1e-9);
    // Away from the pads, nothing is pushed.
    EXPECT_DOUBLE_EQ(
        closingOffset(std::vector<Eigen::Vector3d>{{0.70, 0.05, 0.70}}, tcp, pads), 0.0);
    EXPECT_DOUBLE_EQ(closingOffset({}, tcp, pads), 0.0);
}

TEST(SrdfStates, ReadsTheGroupsNamedStates)
{
    const char * srdf = R"(<robot name="r">
        <group_state name="Default" group="arm">
            <joint name="j1" value="1.5708"/>
            <joint name="j2" value="-0.5"/>
        </group_state>
        <group_state name="open" group="gripper">
            <joint name="f" value="0.04"/>
        </group_state>
        <group_state name="transport" group="arm"><joint name="j1" value="0"/></group_state>
    </robot>)";
    const auto states = namedStates(srdf, "arm");
    ASSERT_EQ(states.size(), 2U);
    EXPECT_DOUBLE_EQ(states.at("Default").at("j2"), -0.5);
    EXPECT_DOUBLE_EQ(states.at("transport").at("j1"), 0.0);
    EXPECT_DOUBLE_EQ(namedStates(srdf, "gripper").at("open").at("f"), 0.04);
}

}  // namespace
}  // namespace piper_skills
