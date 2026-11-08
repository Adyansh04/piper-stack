#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include "piper_skills/place_target.hpp"
#include "piper_skills/view_planner.hpp"

namespace piper_skills
{
namespace
{

TEST(PlaceMode, ParsesTheFourNames)
{
    EXPECT_EQ(parsePlaceMode(""), PlaceMode::kFree);
    EXPECT_EQ(parsePlaceMode("free"), PlaceMode::kFree);
    EXPECT_EQ(parsePlaceMode("relative"), PlaceMode::kRelative);
    EXPECT_EQ(parsePlaceMode("metric"), PlaceMode::kMetric);
    EXPECT_EQ(parsePlaceMode("anchor"), PlaceMode::kAnchor);
    EXPECT_FALSE(parsePlaceMode("Relative"));
}

TEST(PlaceTarget, RelativeScalesToTheTopLessTheInset)
{
    const TableAxes axes;  // x along +y, y along -x, half extents 0.5 and 0.4 m
    const PlaceTarget corner =
        placeTarget(axes, PlaceMode::kRelative, 1.0, -1.0, 0.1, std::nullopt);
    ASSERT_TRUE(corner.point) << corner.why;
    EXPECT_TRUE(corner.point.value_or(Eigen::Vector3d::Constant(std::nan("")))
                    .isApprox(Eigen::Vector3d(0.3, 0.4, 0.0), 1e-12));  // metres
    const PlaceTarget centre = placeTarget(axes, PlaceMode::kRelative, 0.0, 0.0, 0.1, std::nullopt);
    ASSERT_TRUE(centre.point);
    EXPECT_NEAR(centre.point.value_or(Eigen::Vector3d::Constant(std::nan(""))).norm(), 0.0, 1e-12);
    EXPECT_FALSE(placeTarget(axes, PlaceMode::kRelative, 1.2, 0.0, 0.1, std::nullopt).point);
    EXPECT_FALSE(
        placeTarget(axes, PlaceMode::kRelative, std::nan(""), 0.0, 0.1, std::nullopt).point);
}

TEST(PlaceTarget, MetricAndAnchorStayOnTheTop)
{
    const TableAxes axes;
    const PlaceTarget inside = placeTarget(axes, PlaceMode::kMetric, 0.2, 0.1, 0.1, std::nullopt);
    ASSERT_TRUE(inside.point) << inside.why;
    EXPECT_TRUE(inside.point.value_or(Eigen::Vector3d::Constant(std::nan("")))
                    .isApprox(Eigen::Vector3d(-0.1, 0.2, 0.0), 1e-12));
    const PlaceTarget off = placeTarget(axes, PlaceMode::kMetric, 0.45, 0.0, 0.1, std::nullopt);
    EXPECT_FALSE(off.point);
    EXPECT_NE(off.why.find("off the top"), std::string::npos) << off.why;
    // Beside an anchor, at the top's height whatever the anchor's centre height.
    const PlaceTarget beside =
        placeTarget(axes, PlaceMode::kAnchor, 0.15, 0.0, 0.1, Eigen::Vector3d(0.1, 0.0, 0.06));
    ASSERT_TRUE(beside.point) << beside.why;
    EXPECT_TRUE(beside.point.value_or(Eigen::Vector3d::Constant(std::nan("")))
                    .isApprox(Eigen::Vector3d(0.1, 0.15, 0.0), 1e-12));
    EXPECT_FALSE(placeTarget(axes, PlaceMode::kAnchor, 0.15, 0.0, 0.1, std::nullopt).point);
    EXPECT_FALSE(placeTarget(axes, PlaceMode::kFree, 0.0, 0.0, 0.1, std::nullopt).point);
}

TEST(TargetArea, SquareAroundTheTargetOnTheTop)
{
    TopRegion top;
    top.center = Eigen::Vector3d(1.0, 0.0, 0.75);
    const TopRegion area = targetArea(top, Eigen::Vector3d(1.1, 0.2, 0.9), 0.05, 0.12);
    EXPECT_TRUE(area.center.isApprox(Eigen::Vector3d(1.1, 0.2, 0.75), 1e-12));
    EXPECT_NEAR(area.half_width_m, 0.17, 1e-12);  // metres: the tolerance and the free radius
    EXPECT_NEAR(area.half_depth_m, 0.17, 1e-12);
    EXPECT_TRUE(area.lateral.isApprox(top.lateral, 1e-12));
}

TEST(TurnsForYaw, NearestHeadingFirstBothWays)
{
    const double deg = std::numbers::pi / 180.0;
    const std::vector<double> turns = turnsForYaw(0.0, 90.0 * deg, 10.0 * deg, 5.0 * deg);
    ASSERT_EQ(turns.size(), 10U);  // 5 deviations, each either way round
    EXPECT_NEAR(std::abs(turns[0]), std::numbers::pi / 2, 1e-12);  // radians
    EXPECT_NEAR(std::abs(turns[1]), std::numbers::pi / 2, 1e-12);
    EXPECT_NEAR(turns[0] + turns[1], 0.0, 1e-12);
    // A wanted heading half a turn from the held one needs no turn.
    EXPECT_NEAR(turnsForYaw(0.3, 0.3 + std::numbers::pi, 0.0, 0.1).front(), 0.0, 1e-12);
}

TEST(TurnsAfter, TheRestInOrderAndNothingForAnUnknownTurn)
{
    const std::vector<double> turns{0.0, 3.14, 1.57, -1.57};
    EXPECT_EQ(turnsAfter(turns, 3.14), (std::vector<double>{1.57, -1.57}));
    EXPECT_TRUE(turnsAfter(turns, -1.57).empty());
    EXPECT_TRUE(turnsAfter(turns, 0.5).empty());
}

}  // namespace
}  // namespace piper_skills
