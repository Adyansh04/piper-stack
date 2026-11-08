#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <Eigen/Geometry>

namespace piper_skills
{

/** @brief One grasp to choose from, in the robot's base frame (z up). */
struct GraspCandidate
{
    /// Tool centre point at the grasp: z along the approach, y between the fingers.
    Eigen::Isometry3d tcp{Eigen::Isometry3d::Identity()};
    /// The grasp model's confidence, 0 to 1.
    double score{0.0};
    /// Finger opening the grasp needs (twice the farthest object point from the fingers' centre
    /// plane), metres.
    double width_m{0.0};
    /// Width of the object between the fingers, where they stop, metres.
    double object_width_m{0.0};
    /// How far the surfaces the pads meet first lean across them (`contactSpread`), metres.
    double contact_spread_m{0.0};
    /// How far the object reaches back between the pads from the fingertips along the approach
    /// (`graspDepth`), metres; what a side grasp's depth limits apply to.
    double depth_m{0.0};
};

/** @brief Which approaches a ranking takes. */
enum class GraspApproach : std::uint8_t
{
    /// Every approach up to `max_approach_angle_rad` from straight down.
    kAny,
    /// Approaches within `top_max_angle_rad` of straight down.
    kTop,
    /// Flatter approaches: from the side, or diagonally from above.
    kSide
};

/** @brief The finger pads of a parallel gripper. */
struct PadGeometry
{
    /// Width across a pad, metres.
    double width_m{0.0226};
    /// Length of a pad up from the fingertips, metres.
    double length_m{0.065};
    /// Gap between the pads open, metres.
    double open_gap_m{0.0756};
    /// Strips across a pad's width whose outermost object points `contactSpread` compares.
    int strips{4};
};

/** @brief Which grasps the gripper and the table allow, and how to rank them. */
struct GraspFilter
{
    /// Largest angle between the approach and straight down, radians: past pi/2 the approach
    /// rises.
    double max_approach_angle_rad{1.7453};  // 100 degrees, as the YAML
    /// Approaches within this angle of straight down are top grasps, flatter ones side grasps,
    /// radians.
    double top_max_angle_rad{0.8727};  // 50 degrees
    /// A side grasp's closing axis leans at most this far from horizontal: a finger under the
    /// object would meet the table, radians.
    double max_closing_tilt_rad{0.5236};  // 30 degrees
    /// The finger pads, and the thickness of a finger outside its pad along the closing axis,
    /// metres: together the fingers a side grasp keeps above the top.
    PadGeometry pads{};
    double finger_thickness_m{0.029};
    /// The object between the fingers is at least this wide (the closed gap and a margin), and the
    /// opening a grasp needs at most this wide (the open gap less a clearance), metres.
    double min_width_m{0.01};
    double max_width_m{0.06};
    /// The TCP (the fingertips) stays this far above the top, metres.
    double top_clearance_m{0.01};
    /// The fingertips reach at least this far below the object's top, metres, or
    /// `min_depth_share` of the object's height when that is less.
    double min_depth_m{0.03};
    double min_depth_share{0.5};
    /// The fingertips reach at most this far below the object's top, so the finger roots stay above
    /// it, metres.
    double max_depth_m{0.055};
    /// A side grasp grips (the middle of the object between its pads) at most this share of the
    /// object's height above or below its mid-height: a grip near an edge lets the object turn out
    /// of the pads when the arm turns the gripper into its transport pose.
    double side_max_offset_share{0.25};
    /// Weight of an upright approach against the model's confidence in the rank.
    double upright_weight{0.5};
    /// Weight of the fingertips' depth below the object's top in the rank, per metre.
    double depth_weight{10.0};
    /// Weight of a side grasp's depth into the object in the rank, per metre, in place of
    /// `depth_weight`: a grip short of the object's middle along the approach lets it turn out.
    double side_depth_weight{40.0};
    /// Weight of the opening a grasp needs in the rank, per metre: the narrowest, centred way across
    /// a box closes on two faces, a wider one on corners the object turns on.
    double width_weight{20.0};
    /// Weight of the pads' contact spread in the rank, per metre: a pad meeting a face square holds,
    /// one meeting an edge at an angle turns the object out (a long part crossed at a slant).
    double contact_weight{50.0};
};

/** @brief What the filter made of one grasp: ranked, or the first limit it failed. */
enum class GraspVerdict : std::uint8_t
{
    /// Passed every limit: in the ranking.
    kRanked,
    /// The approach is further from straight down than `max_approach_angle_rad`.
    kTooTilted,
    /// The object between the fingers is narrower than `min_width_m`.
    kTooNarrow,
    /// The grasp needs an opening wider than `max_width_m`.
    kTooWide,
    /// The fingertips (a side grasp: any point of the fingers) come nearer the top than
    /// `top_clearance_m`.
    kTooLow,
    /// The fingertips stay above the least depth below the object's top (a side grasp: reach less
    /// far into the object).
    kTooShallow,
    /// The fingertips reach deeper below the object's top (a side grasp: further into the object)
    /// than `max_depth_m`.
    kTooDeep,
    /// The approach is not of the kind the ranking takes (top or side).
    kOtherApproach,
    /// A side grasp's closing axis leans further from horizontal than `max_closing_tilt_rad`.
    kFingerUnder,
    /// A side grasp grips further from the object's mid-height than `side_max_offset_share` of its
    /// height.
    kOffCentre
};

/** @brief Grasps that passed, best first, and how many each limit rejected. */
struct GraspRanking
{
    /// Indices into the candidates.
    std::vector<std::size_t> order;
    /// Per candidate, in the candidates' order.
    std::vector<GraspVerdict> verdicts;
    /// Rejected candidates by the first limit they failed (see GraspVerdict).
    std::size_t too_tilted{0};
    /// See GraspVerdict::kTooNarrow.
    std::size_t too_narrow{0};
    /// See GraspVerdict::kTooWide.
    std::size_t too_wide{0};
    /// See GraspVerdict::kTooLow.
    std::size_t too_low{0};
    /// See GraspVerdict::kTooShallow.
    std::size_t too_shallow{0};
    /// See GraspVerdict::kTooDeep.
    std::size_t too_deep{0};
    /// See GraspVerdict::kOtherApproach.
    std::size_t other_approach{0};
    /// See GraspVerdict::kFingerUnder.
    std::size_t finger_under{0};
    /// See GraspVerdict::kOffCentre.
    std::size_t off_centre{0};
};

/**
 * @brief Whether a grasp approaches from the side (or diagonally) rather than from above.
 * @param tcp The grasp's TCP pose, base frame (z up).
 * @param filter Holds `top_max_angle_rad`.
 * @return True when the approach is further than `top_max_angle_rad` from straight down.
 */
[[nodiscard]] bool isSideGrasp(const Eigen::Isometry3d & tcp, const GraspFilter & filter);

/**
 * @brief Grasps the gripper and the table allow, best first.
 * @details A top grasp's depth is the fingertips' depth below the object's top; a side grasp's is
 * the candidate's `depth_m`, all of its fingers (pads and finger thickness) stay
 * `top_clearance_m` above the top, and it grips within `side_max_offset_share` of the object's
 * height of its mid-height.
 * @param grasps Candidates.
 * @param filter Limits and ranking weights.
 * @param top_height_m Height of the surface the object stands on, base frame, metres.
 * @param object_top_m Height of the object's highest point, base frame, metres.
 * @param approach Which approaches to rank; the others are counted as `other_approach`.
 * @return The passing grasps ranked by the confidence, plus `upright_weight` times the cosine of the
 * approach angle, plus `depth_weight` (a side grasp: `side_depth_weight`) times the depth, less
 * `width_weight` times the opening the grasp needs and `contact_weight` times its contact spread.
 */
[[nodiscard]] GraspRanking rankGrasps(
    const std::vector<GraspCandidate> & grasps,
    const GraspFilter & filter,
    double top_height_m,
    double object_top_m,
    GraspApproach approach = GraspApproach::kAny);

/**
 * @brief How far an object reaches back between a grasp's pads from the fingertips.
 * @param points Object points, metres.
 * @param tcp The grasp's TCP pose in the points' frame (y between the fingers, z the approach).
 * @param pads Pad size and open gap.
 * @return The largest distance behind the TCP along the approach of a point between the open pads
 * (within the pads' width and length), metres; 0 without such a point.
 */
[[nodiscard]] double graspDepth(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads);

/**
 * @brief The lowest point of a grasp's fingers: both pads and the finger thickness outside them.
 * @param tcp The grasp's TCP pose, base frame (z up).
 * @param pads Pad size and open gap.
 * @param finger_thickness_m Finger thickness outside a pad along the closing axis, metres.
 * @return Its height, base frame, metres.
 */
[[nodiscard]] double lowestFingerZ(
    const Eigen::Isometry3d & tcp, const PadGeometry & pads, double finger_thickness_m);

/**
 * @brief How far the surfaces a grasp's finger pads meet first lean across the pads.
 * @details Per side of the fingers' centre plane, the outermost object point between the open
 * pads in each strip across their width; the result is the larger spread of those along the
 * closing axis over the two sides. It is about 0 on a face parallel to the pads and grows towards
 * the pads' width times the tangent of the angle on an edge the grasp crosses at a slant.
 * @param points Object points, metres.
 * @param tcp The grasp's TCP pose in the points' frame (y between the fingers, z the approach).
 * @param pads Pad size and open gap.
 * @return The spread in metres; 0 when no side has points in two strips.
 */
[[nodiscard]] double contactSpread(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads);

/**
 * @brief How far off the fingers' centre plane the object's part between the pads lies.
 * @details Closing, the pads push that part onto the plane: the whole object slides this far back
 * along the closing axis.
 * @param points Object points, metres.
 * @param tcp The grasp's TCP pose in the points' frame (x across the pads, y the closing axis, z the approach).
 * @param pads The finger pads.
 * @return The middle of those points along the TCP's y axis, metres; 0 without any.
 */
[[nodiscard]] double closingOffset(
    std::span<const Eigen::Vector3d> points,
    const Eigen::Isometry3d & tcp,
    const PadGeometry & pads);

/**
 * @brief The pose `distance_m` back from a TCP pose along its approach.
 * @param tcp TCP pose.
 * @param distance_m Distance, metres.
 * @return The pre-grasp pose (same orientation).
 */
[[nodiscard]] Eigen::Isometry3d backOff(const Eigen::Isometry3d & tcp, double distance_m);

}  // namespace piper_skills
