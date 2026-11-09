#include "piper_skills/stage_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <expected>
#include <format>
#include <future>
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
            });
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
                break;
        }
        return {};
    };
    return motion.toJoints(*above)
        .and_then([&] { return allowed.add(stage.contacts); })
        .and_then([&] { return stopIf(cancelled); })
        .and_then([&] {
            phase("inserting");
            return motion.linear(start * grasp.in_part, settings_.insert_velocity_scale);
        })
        .and_then(finish)
        .and_then([&] {
            phase("releasing");
            return motion.gripper(stage.opening_m, settings_.effort).transform([](double) {});
        })
        .and_then([&]() -> Status {
            return stage.motion == StageMotion::kScrew ? Status() : attach(stage.part, false);
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
    Eigen::Isometry3d grasp,
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
    const double sign =
        ahead->at(settings_.wrist_joint) >= now->at(settings_.wrist_joint) ? 1.0 : -1.0;
    double left = from;
    while (left > 0.0)
    {
        if (cancelled())
        {
            return std::unexpected("cancelled");
        }
        const auto joints = motion.joints();
        if (!joints)
        {
            return std::unexpected("no current joint positions");
        }
        const double wrist = joints->at(settings_.wrist_joint);
        const double room_rad = sign > 0.0 ? high - wrist : wrist - low;
        if (room_rad >= kMinTurnRad)
        {
            phase("screwing");
            const double next = std::max(0.0, left - (room_rad / whole_rad));
            if (auto turned = turn(motion, stage, mate, grasp, left, next); !turned)
            {
                return turned;
            }
            left = next;
            continue;
        }
        phase("regrasping");
        if (auto opened = motion.gripper(stage.opening_m, settings_.effort); !opened)
        {
            return std::unexpected(opened.error());
        }
        JointValues back = *joints;
        back[settings_.wrist_joint] -= sign * std::numbers::pi;
        if (auto turned = motion.toJoints(back); !turned)
        {
            return turned;
        }
        if (auto closed = close(motion); !closed)
        {
            return closed;
        }
        // Half a turn back on a part with two-fold flats: the grasp turned by half a turn.
        grasp = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ()) * grasp;
    }
    return {};
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
    const auto squeezed = motion.gripper(
        std::max(settings_.closed_m, *finger - settings_.squeeze_m), settings_.effort);
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
