#include "piper_skills/stage_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <expected>
#include <format>
#include <future>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "piper_skills/arm_motion.hpp"
#include "piper_skills/scene_client.hpp"
#include "piper_skills/srdf_states.hpp"
#include "piper_skills/task_stages.hpp"

namespace piper_skills
{
namespace
{

// The wrist turns at least this far between two regrasps of a screw, radians.
constexpr double kMinTurnRad = 0.1;
// How far along a screw the TCP is probed to learn which way the wrist turns, radians.
constexpr double kProbeTurnRad = 0.2;

/// A part's pose backed off from its mate along the stage's approach axis.
Eigen::Isometry3d backedOff(const Stage & stage, const Eigen::Isometry3d & pose, double distance_m)
{
    Eigen::Isometry3d out = pose;
    out.translate(distance_m * stage.approach_axis);
    return out;
}

template <typename Service>
std::expected<void, std::string> callService(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const std::shared_ptr<typename Service::Request> & request,
    std::chrono::milliseconds timeout)
{
    if (!client->wait_for_service(timeout))
    {
        return std::unexpected(
            std::format("{} is not offered: is task_scene running?", client->get_service_name()));
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready)
    {
        client->remove_pending_request(future);
        return std::unexpected(std::format("{} did not answer", client->get_service_name()));
    }
    const auto response = future.get();
    if (!response->success)
    {
        return std::unexpected(
            std::format("{}: {}", client->get_service_name(), response->message));
    }
    return {};
}

std::expected<void, std::string> stopIf(const StageRunner::Cancelled & cancelled)
{
    if (cancelled())
    {
        return std::unexpected("cancelled");
    }
    return {};
}

/// The poses of a stage's part and mate in MoveIt's scene.
std::expected<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>, std::string> locate(
    const SceneClient & scene, const Stage & stage)
{
    const auto poses = scene.objectPoses({stage.part, stage.mate});
    if (!poses)
    {
        return std::unexpected(poses.error());
    }
    for (const auto & name : {stage.part, stage.mate})
    {
        if (!poses->contains(name))
        {
            return std::unexpected(
                std::format(
                    "MoveIt's scene has no {} in the world: is task_scene running, or is it "
                    "still held?",
                    name));
        }
    }
    return std::pair{poses->at(stage.part), poses->at(stage.mate)};
}

}  // namespace

StageRunner::AllowedContacts::AllowedContacts(StageRunner & runner) : runner_(runner) {}

StageRunner::AllowedContacts::~AllowedContacts()
{
    try
    {
        if (const auto forbidden = runner_.allow(pairs_, false); !forbidden)
        {
            RCLCPP_WARN(
                runner_.logger_,
                "The stage's contacts stay allowed: %s",
                forbidden.error().c_str());
        }
    }
    catch (const std::exception & error)
    {
        RCLCPP_WARN(runner_.logger_, "The stage's contacts stay allowed: %s", error.what());
    }
}

StageRunner::Status StageRunner::AllowedContacts::add(const Pairs & pairs)
{
    auto allowed = runner_.allow(pairs, true);
    if (allowed)
    {
        pairs_.insert(pairs_.end(), pairs.begin(), pairs.end());
    }
    return allowed;
}

StageRunner::Status StageRunner::AllowedContacts::remove(const Pairs & pairs)
{
    auto forbidden = runner_.allow(pairs, false);
    if (forbidden)
    {
        std::erase_if(pairs_, [&pairs](const auto & pair) {
            return std::ranges::find(pairs, pair) != pairs.end();
        });
    }
    return forbidden;
}

StageRunner::StageRunner(
    rclcpp::Node & node,
    const rclcpp::CallbackGroup::SharedPtr & group,
    std::shared_ptr<const SceneClient> scene,
    std::vector<Stage> stages,
    StageSettings settings)
    : scene_(std::move(scene))
    , stages_(std::move(stages))
    , settings_(std::move(settings))
    , logger_(node.get_logger())
    , attach_client_(node.create_client<piper_msgs::srv::AttachPart>(
          settings_.scene_node + "/attach", rclcpp::ServicesQoS(), group))
    , allow_client_(node.create_client<piper_msgs::srv::AllowContacts>(
          settings_.scene_node + "/allow_contacts", rclcpp::ServicesQoS(), group))
{}

std::expected<double, std::string> StageRunner::run(
    ArmMotion & motion, const std::string & name, const Phase & phase, const Cancelled & cancelled)
{
    const auto found = std::ranges::find(stages_, name, &Stage::name);
    if (found == stages_.end())
    {
        return std::unexpected(std::format("the task has no stage {}", name));
    }
    const Stage & stage = *found;
    phase("locating");
    const auto located = locate(*scene_, stage);
    if (!located)
    {
        return std::unexpected(located.error());
    }
    const auto & [part, mate] = *located;
    AllowedContacts allowed(*this);
    Pairs touching;
    for (const auto & link : settings_.touch_links)
    {
        touching.emplace_back(link, stage.part);
    }
    // Held, the part still touches what it rests on until it is lifted off it, and so may the
    // fingers round it (a cradle or a pocket's rim).
    Pairs resting;
    if (!stage.support.empty())
    {
        resting.emplace_back(stage.part, stage.support);
        for (const auto & link : settings_.touch_links)
        {
            resting.emplace_back(link, stage.support);
        }
    }
    touching.insert(touching.end(), resting.begin(), resting.end());
    const auto grasp =
        allowed.add(touching)
            .and_then([&] { return pick(motion, stage, part, mate, phase, cancelled); })
            .and_then([&](const Grasp & held) {
                return allowed.remove(resting).transform([&] { return held; });
            })
            .and_then([&](Grasp held) { return inHand(motion, stage, std::move(held)); });
    if (!grasp)
    {
        return std::unexpected(grasp.error());
    }
    if (auto placed = place(motion, stage, mate, *grasp, allowed, phase, cancelled); !placed)
    {
        return std::unexpected(placed.error());
    }
    phase("checking");
    return check(stage);
}

std::expected<StageRunner::Grasp, std::string> StageRunner::pick(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & part,
    const Eigen::Isometry3d & mate,
    const Phase & phase,
    const Cancelled & cancelled) const
{
    const auto grasp = chooseGrasp(motion, stage, part, mate);
    if (!grasp)
    {
        return std::unexpected(grasp.error());
    }
    phase("grasping");
    const Eigen::Isometry3d tcp = part * grasp->in_part;
    Eigen::Isometry3d lifted = tcp;
    lifted.translation().z() += settings_.lift_m;
    // Each step runs once the one before it has succeeded.
    const Status done =
        motion.gripper(stage.opening_m, settings_.effort)
            .transform([](double) {})
            .and_then([&] { return motion.toJoints(grasp->pregrasp); })
            .and_then([&] { return stopIf(cancelled); })
            .and_then([&] { return motion.linear(tcp, settings_.insert_velocity_scale); })
            .and_then([&] { return close(motion); })
            .and_then([&] { return attach(stage.part, true); })
            .and_then([&] {
                phase("lifting");
                return motion.linear(lifted);
            })
            .and_then([&]() -> Status {
                const auto finger = motion.fingerTravel();
                if (!finger || *finger <= settings_.held_min_m)
                {
                    return std::unexpected(
                        std::format("{} slipped out of the fingers while lifting", stage.part));
                }
                return {};
            });
    if (!done)
    {
        return std::unexpected(done.error());
    }
    return *grasp;
}

std::expected<StageRunner::Grasp, std::string> StageRunner::inHand(
    ArmMotion & motion, const Stage & stage, Grasp grasp) const
{
    // Let go of in the scene, the part goes back to where it was last measured.
    if (auto released = attach(stage.part, false); !released)
    {
        return std::unexpected(released.error());
    }
    std::this_thread::sleep_for(settings_.scene_settle);
    const auto poses = scene_->objectPoses({stage.part});
    if (auto held = attach(stage.part, true); !held)
    {
        return std::unexpected(held.error());
    }
    if (!poses || !poses->contains(stage.part))
    {
        return std::unexpected(
            std::format("{} is not in MoveIt's scene to see in the fingers", stage.part));
    }
    const Eigen::Isometry3d in_part = poses->at(stage.part).inverse() * motion.tcpPose();
    const Eigen::Isometry3d off = grasp.in_part.inverse() * in_part;
    RCLCPP_INFO(
        logger_,
        "The %s sits %.1f mm and %.0f degrees off its grasp",
        stage.part.c_str(),
        1000.0 * off.translation().norm(),
        Eigen::AngleAxisd(off.linear()).angle() * 180.0 / std::numbers::pi);
    grasp.in_part = in_part;
    return grasp;
}

StageRunner::Status StageRunner::place(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & mate,
    const Grasp & grasp,
    AllowedContacts & allowed,
    const Phase & phase,
    const Cancelled & cancelled) const
{
    phase("approaching");
    const Eigen::Isometry3d start = mate * partOnMate(stage, grasp.start_left);
    const auto above = motion.ik(backedOff(stage, start, stage.approach_m) * grasp.in_part);
    if (!above)
    {
        return std::unexpected(std::format("above {}'s mate: {}", stage.part, above.error()));
    }
    const auto finish = [&]() -> Status {
        switch (stage.motion)
        {
            case StageMotion::kScrew:
                // The thread holds the part from here: it turns as part of the world, the fingers
                // on it.
                return attach(stage.part, false).and_then([&] {
                    return screw(
                        motion, stage, mate, grasp.in_part, grasp.start_left, phase, cancelled);
                });
            case StageMotion::kTwist:
                phase("twisting");
                return turn(motion, stage, mate, grasp.in_part, 1.0, 0.0);
            case StageMotion::kInsert:
                if (stage.guide.has_value())
                {
                    const InsertGuide & guide = *stage.guide;
                    // Seen in the scene from here, the part moves as part of the world.
                    return attach(stage.part, false).and_then([&] {
                        return guided(motion, stage, guide, mate, grasp.in_part, cancelled);
                    });
                }
                break;
        }
        return {};
    };
    // A guided insert comes straight in only to where its guided moves start.
    const Eigen::Isometry3d in_to =
        stage.guide ? backedOff(stage, start, stage.guide->from_m) : start;
    return motion.toJoints(*above)
        .and_then([&] { return allowed.add(stage.contacts); })
        .and_then([&] { return stopIf(cancelled); })
        .and_then([&] {
            phase("inserting");
            return motion.linear(in_to * grasp.in_part, settings_.insert_velocity_scale);
        })
        .and_then(finish)
        .and_then([&] {
            phase("releasing");
            return motion.gripper(stage.opening_m, settings_.effort).transform([](double) {});
        })
        .and_then([&]() -> Status {
            const bool detached = stage.motion == StageMotion::kScrew || stage.guide;
            return detached ? Status() : attach(stage.part, false);
        })
        .and_then([&] {
            phase("retreating");
            Eigen::Isometry3d clear = motion.tcpPose();
            clear.translation().z() += stage.approach_m;
            return motion.linear(clear);
        });
}

std::expected<double, std::string> StageRunner::check(const Stage & stage) const
{
    std::this_thread::sleep_for(settings_.settle);
    const auto ended = locate(*scene_, stage);
    if (!ended)
    {
        return std::unexpected(ended.error());
    }
    const Eigen::Isometry3d goal = ended->second * stage.goal;
    const Eigen::Isometry3d & now = ended->first;
    const double error_m = (now.translation() - goal.translation()).norm();
    const double tilt_rad =
        std::acos(std::clamp(goal.linear().col(2).dot(now.linear().col(2)), -1.0, 1.0));
    // The turn about the goal's z, up to the part's symmetry; it counts where the stage turns.
    const Eigen::Vector3d x_in_goal = goal.linear().transpose() * now.linear().col(0);
    const double period_rad = 2.0 * std::numbers::pi / stage.symmetry;
    const double yaw_rad = std::remainder(std::atan2(x_in_goal.y(), x_in_goal.x()), period_rad);
    const bool turns = stage.motion != StageMotion::kInsert;
    const double tolerance_m =
        stage.goal_tolerance_m > 0.0 ? stage.goal_tolerance_m : settings_.goal_tolerance_m;
    const double tolerance_rad =
        stage.goal_tolerance_rad > 0.0 ? stage.goal_tolerance_rad : settings_.goal_tolerance_rad;
    // A part with no symmetry about its axis (a plug) must end turned right even when inserted.
    const bool aligned = stage.symmetry == 1 || turns;
    if (error_m > tolerance_m || tilt_rad > tolerance_rad
        || (aligned && std::abs(yaw_rad) > tolerance_rad))
    {
        return std::unexpected(
            std::format(
                "{} ended {:.1f} mm, tilted {:.0f} and turned {:.0f} degrees from its goal",
                stage.part,
                1000.0 * error_m,
                tilt_rad * 180.0 / std::numbers::pi,
                yaw_rad * 180.0 / std::numbers::pi));
    }
    RCLCPP_INFO(
        logger_,
        "Stage %s: %s %.1f mm and %.0f degrees from its goal",
        stage.name.c_str(),
        stage.part.c_str(),
        1000.0 * error_m,
        yaw_rad * 180.0 / std::numbers::pi);
    return error_m;
}

std::expected<StageRunner::Grasp, std::string> StageRunner::chooseGrasp(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & part,
    const Eigen::Isometry3d & mate) const
{
    // A screw may start up to a turn higher: each half turn turns the part, and the TCP, by pi.
    std::vector<double> starts{1.0};
    if (stage.motion == StageMotion::kScrew)
    {
        starts = {1.0, (stage.turns + 0.5) / stage.turns, (stage.turns + 1.0) / stage.turns};
    }
    std::string why;
    for (const auto & grasp : stageGrasps(stage))
    {
        // Back along the TCP's z, which points into the part.
        const Eigen::Isometry3d pregrasp =
            part * grasp * Eigen::Translation3d(0.0, 0.0, -settings_.pregrasp_m);
        auto joints = motion.ik(pregrasp);
        if (!joints)
        {
            why = std::format("above {}: {}", stage.part, joints.error());
            continue;
        }
        for (const double start : starts)
        {
            const Eigen::Isometry3d above =
                backedOff(stage, mate * partOnMate(stage, start), stage.approach_m);
            if (auto over = motion.ik(above * grasp); !over)
            {
                why = std::format("above its mate: {}", over.error());
                continue;
            }
            return Grasp{.in_part = grasp, .pregrasp = std::move(*joints), .start_left = start};
        }
    }
    return std::unexpected(
        std::format("no grasp of {} reaches it and its mate ({})", stage.part, why));
}

StageRunner::Status StageRunner::turn(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & mate,
    const Eigen::Isometry3d & grasp,
    double from,
    double to) const
{
    const double turn_rad = std::abs((from - to) * stageTurn(stage));
    const int steps =
        std::max(1, static_cast<int>(std::ceil(turn_rad / settings_.max_turn_step_rad)));
    for (int i = 1; i <= steps; ++i)
    {
        const double left = from + ((to - from) * i / steps);
        const Eigen::Isometry3d tcp = mate * partOnMate(stage, left) * grasp;
        if (auto moved = motion.linear(tcp, settings_.turn_velocity_scale); !moved)
        {
            return moved;
        }
    }
    return {};
}

StageRunner::Status StageRunner::screw(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & mate,
    const Eigen::Isometry3d & grasp,
    double from,
    const Phase & phase,
    const Cancelled & cancelled) const
{
    const auto limits = motion.jointLimits(settings_.wrist_joint);
    if (!limits)
    {
        return std::unexpected(std::format("the robot has no joint {}", settings_.wrist_joint));
    }
    const double low = limits->first + settings_.wrist_margin_rad;
    const double high = limits->second - settings_.wrist_margin_rad;
    const double whole_rad = std::abs(stageTurn(stage));
    // Which way the wrist turns with the part: IK of the TCP a little further down the thread.
    const auto now = motion.joints();
    const auto ahead =
        motion.ik(mate * partOnMate(stage, from - (kProbeTurnRad / whole_rad)) * grasp);
    if (!now || !ahead || !now->contains(settings_.wrist_joint))
    {
        return std::unexpected("no wrist position to screw from");
    }
    const double wrist_sign =
        ahead->at(settings_.wrist_joint) >= now->at(settings_.wrist_joint) ? 1.0 : -1.0;
    const Eigen::Isometry3d seat = mate * stage.goal;
    // The TCP on the part as the fingers hold it, so a part that sat in them off the grasp is
    // not pulled back to it.
    const auto start = onSeat(stage, seat);
    if (!start)
    {
        return std::unexpected(start.error());
    }
    Eigen::Isometry3d held = (seat * *start).inverse() * motion.tcpPose();
    const double budget_rad =
        ((from * stage.turns) + settings_.screw_search_turns) * 2.0 * std::numbers::pi;
    double turned_rad = 0.0;
    phase("screwing");
    while (true)
    {
        if (cancelled())
        {
            return std::unexpected("cancelled");
        }
        const auto on_seat = onSeat(stage, seat);
        if (!on_seat)
        {
            return std::unexpected(on_seat.error());
        }
        const double height_m = on_seat->translation().z();
        if (height_m <= settings_.screw_seated_m)
        {
            return {};
        }
        if (turned_rad >= budget_rad)
        {
            return std::unexpected(
                std::format(
                    "the thread did not take {}: {:.1f} mm above its seat after {:.1f} turns",
                    stage.part,
                    1000.0 * height_m,
                    turned_rad / (2.0 * std::numbers::pi)));
        }
        const auto joints = motion.joints();
        if (!joints)
        {
            return std::unexpected("no current joint positions");
        }
        const double wrist = joints->at(settings_.wrist_joint);
        const double room_rad = wrist_sign > 0.0 ? high - wrist : wrist - low;
        if (room_rad < kMinTurnRad)
        {
            phase("regrasping");
            auto regrasped = regrasp(motion, stage, seat, *joints, wrist_sign);
            if (!regrasped)
            {
                return std::unexpected(regrasped.error());
            }
            held = *regrasped;
            phase("screwing");
            continue;
        }
        const double step_rad = std::min(settings_.screw_step_rad, room_rad);
        const Eigen::Isometry3d target =
            seat * screwStep(stage, *on_seat, step_rad, settings_.screw_preload_m) * held;
        if (auto moved = motion.linear(target, settings_.turn_velocity_scale); !moved)
        {
            return moved;
        }
        turned_rad += step_rad;
    }
}

std::expected<Eigen::Isometry3d, std::string> StageRunner::regrasp(
    ArmMotion & motion,
    const Stage & stage,
    const Eigen::Isometry3d & seat,
    const JointValues & joints,
    double wrist_sign) const
{
    if (auto opened = motion.gripper(stage.opening_m, settings_.effort); !opened)
    {
        return std::unexpected(opened.error());
    }
    JointValues back = joints;
    back[settings_.wrist_joint] -= wrist_sign * std::numbers::pi;
    if (auto turned = motion.toJoints(back); !turned)
    {
        return std::unexpected(turned.error());
    }
    // The fingers go back to the grasp's height on the part however it moved in them, the grasp
    // of its symmetry nearest the turned TCP.
    const auto on_seat = onSeat(stage, seat);
    if (!on_seat)
    {
        return std::unexpected(on_seat.error());
    }
    const Eigen::Isometry3d part = seat * *on_seat;
    const Eigen::Isometry3d tcp = motion.tcpPose();
    const auto grasps = stageGrasps(stage);
    const auto nearest = std::ranges::min_element(grasps, {}, [&](const Eigen::Isometry3d & grasp) {
        return Eigen::AngleAxisd((part * grasp).linear().transpose() * tcp.linear()).angle();
    });
    if (auto moved = motion.linear(part * *nearest, settings_.insert_velocity_scale); !moved)
    {
        return std::unexpected(moved.error());
    }
    if (auto closed = close(motion); !closed)
    {
        return std::unexpected(closed.error());
    }
    const auto taken = onSeat(stage, seat);
    if (!taken)
    {
        return std::unexpected(taken.error());
    }
    return (seat * *taken).inverse() * motion.tcpPose();
}

std::expected<Eigen::Isometry3d, std::string> StageRunner::onSeat(
    const Stage & stage, const Eigen::Isometry3d & seat) const
{
    std::this_thread::sleep_for(settings_.scene_settle);
    const auto poses = scene_->objectPoses({stage.part});
    if (!poses)
    {
        return std::unexpected(poses.error());
    }
    if (!poses->contains(stage.part))
    {
        return std::unexpected(std::format("MoveIt's scene has no {} in the world", stage.part));
    }
    const Eigen::Isometry3d in_seat = seat.inverse() * poses->at(stage.part);
    // Its tilt and its offset from the axis are left out: the next move rights and centres it.
    Eigen::Isometry3d upright = Eigen::Isometry3d::Identity();
    upright.translate(Eigen::Vector3d(0.0, 0.0, in_seat.translation().z()));
    upright.rotate(
        Eigen::AngleAxisd(
            std::atan2(in_seat.linear()(1, 0), in_seat.linear()(0, 0)), Eigen::Vector3d::UnitZ()));
    return upright;
}

StageRunner::Status StageRunner::guided(
    ArmMotion & motion,
    const Stage & stage,
    const InsertGuide & guide,
    const Eigen::Isometry3d & mate,
    const Eigen::Isometry3d & held,
    const Cancelled & cancelled) const
{
    const double tolerance_m =
        stage.goal_tolerance_m > 0.0 ? stage.goal_tolerance_m : settings_.goal_tolerance_m;
    const Eigen::Isometry3d goal = mate * partOnMate(stage, 0.0);
    // The part `left_m` from its goal along the approach axis and `off` across it, centred and
    // turned as the goal of its symmetry nearest how it sits now.
    const auto move_to =
        [&](const Eigen::Isometry3d & in_goal, double left_m, const Eigen::Vector3d & off) {
            Eigen::Isometry3d target = goal;
            target.translate((left_m * stage.approach_axis) + off);
            if (stage.symmetry == 2 && in_goal.linear()(0, 0) < 0.0)
            {
                target.rotate(Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ()));
            }
            return motion.linear(target * held, settings_.insert_velocity_scale);
        };
    double before_m = std::numeric_limits<double>::infinity();
    int tries = 0;
    Eigen::Vector3d off = Eigen::Vector3d::Zero();
    const int moves =
        static_cast<int>(std::ceil(guide.from_m / guide.step_m)) + (2 * guide.tries) + 2;
    for (int move = 0; move < moves; ++move)
    {
        if (cancelled())
        {
            return std::unexpected("cancelled");
        }
        std::this_thread::sleep_for(settings_.scene_settle);
        const auto poses = scene_->objectPoses({stage.part});
        if (!poses || !poses->contains(stage.part))
        {
            return std::unexpected(
                std::format("MoveIt's scene has no {} in the world", stage.part));
        }
        const Eigen::Isometry3d in_goal = goal.inverse() * poses->at(stage.part);
        const double left_m = in_goal.translation().dot(stage.approach_axis);
        if (left_m <= tolerance_m)
        {
            return {};
        }
        Status moved;
        if (before_m - left_m < guide.step_m / 2.0)
        {
            if (++tries > guide.tries)
            {
                return std::unexpected(
                    std::format(
                        "{} stuck {:.1f} mm short of its goal after {} tries",
                        stage.part,
                        1000.0 * left_m,
                        guide.tries));
            }
            // Back off a step and come in again at the next point of a spiral round the axis.
            off = searchOffset(stage, guide, tries);
            moved = move_to(in_goal, left_m + guide.step_m, off);
            before_m = std::numeric_limits<double>::infinity();
        }
        else
        {
            moved = move_to(in_goal, std::max(0.0, left_m - guide.step_m) - guide.push_m, off);
            before_m = left_m;
        }
        if (!moved)
        {
            return moved;
        }
    }
    return std::unexpected(std::format("{} did not reach its goal in {} moves", stage.part, moves));
}

StageRunner::Status StageRunner::close(ArmMotion & motion) const
{
    const auto finger = motion.gripper(settings_.closed_m, settings_.effort);
    if (!finger)
    {
        return std::unexpected(finger.error());
    }
    RCLCPP_INFO(logger_, "The fingers closed %.1f mm apart", 2000.0 * *finger);
    if (*finger <= settings_.held_min_m)
    {
        return std::unexpected(
            std::format("the fingers closed on nothing ({:.1f} mm apart)", 2000.0 * *finger));
    }
    // At most half its width further in: a thin part (a plug) is not squeezed to nothing.
    const auto squeezed = motion.gripper(
        std::max(settings_.closed_m, *finger - std::min(settings_.squeeze_m, *finger / 2.0)),
        settings_.effort);
    if (!squeezed)
    {
        return std::unexpected(squeezed.error());
    }
    return {};
}

StageRunner::Status StageRunner::allow(const Pairs & pairs, bool allowed) const
{
    if (pairs.empty())
    {
        return {};
    }
    auto request = std::make_shared<piper_msgs::srv::AllowContacts::Request>();
    for (const auto & [first, second] : pairs)
    {
        request->first.push_back(first);
        request->second.push_back(second);
    }
    request->allowed = allowed;
    return callService<piper_msgs::srv::AllowContacts>(
        allow_client_, request, settings_.service_timeout);
}

StageRunner::Status StageRunner::attach(const std::string & part, bool attach) const
{
    auto request = std::make_shared<piper_msgs::srv::AttachPart::Request>();
    request->part = part;
    request->attach = attach;
    return callService<piper_msgs::srv::AttachPart>(
        attach_client_, request, settings_.service_timeout);
}

}  // namespace piper_skills
