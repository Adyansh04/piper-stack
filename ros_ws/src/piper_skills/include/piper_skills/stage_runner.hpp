#pragma once

#include <chrono>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <piper_msgs/srv/allow_contacts.hpp>
#include <piper_msgs/srv/attach_part.hpp>
#include <rclcpp/rclcpp.hpp>

#include "piper_skills/arm_motion.hpp"
#include "piper_skills/scene_client.hpp"
#include "piper_skills/task_stages.hpp"

namespace piper_skills
{

/**
 * @brief How a task's stages are run.
 */
struct StageSettings
{
    /**
     * @brief The node keeping the task's parts in MoveIt's scene (task_scene), whose `attach` and
     * `allow_contacts` services the stages call.
     */
    std::string scene_node{"/task_scene"};

    /**
     * @brief The robot links a held part may touch: the gripper's.
     */
    std::vector<std::string> touch_links;

    /**
     * @brief Each finger's travel when closed, and the effort it closes with (the gripper
     * controller's units).
     */
    double closed_m{0.0};
    double effort{1.0};

    /**
     * @brief A finger that stops at or below this travel holds nothing, metres.
     */
    double held_min_m{0.002};

    /**
     * @brief Once the fingers stop on a part they are sent this much further in, metres: a squeeze
     * within the finger drives' force limit, so they keep the part centred between them.
     */
    double squeeze_m{0.003};

    /**
     * @brief How far above a grasp the fingers come down from, and how far a grasped part is
     * lifted, metres.
     */
    double pregrasp_m{0.06};
    double lift_m{0.06};

    /**
     * @brief Speed scalings, 0 to 1, of the moves onto the mate and of the turns.
     */
    double insert_velocity_scale{0.1};
    double turn_velocity_scale{0.6};

    /**
     * @brief The largest turn of one straight move, radians.
     */
    double max_turn_step_rad{1.5708};

    /**
     * @brief The joint that turns the TCP about its z, and how far from its limits a screw stops
     * to regrasp, radians.
     */
    std::string wrist_joint{"joint6"};
    double wrist_margin_rad{0.15};

    /**
     * @brief How long a released part settles before its pose is checked, and how far from its
     * goal it may end: metres, and radians about any axis.
     */
    std::chrono::milliseconds settle{1000};
    double goal_tolerance_m{0.003};
    double goal_tolerance_rad{0.35};

    /**
     * @brief The longest wait for task_scene's services.
     */
    std::chrono::milliseconds service_timeout{3000};
};

/**
 * @class StageRunner
 * @brief Runs the stages of a task (`task.yaml`) with the arm, through MoveIt.
 * @details A stage takes its part where MoveIt's scene has it (task_scene keeps the parts' poses
 * there), grasps it with the grasp of its symmetry that the arm reaches at the part and above its
 * mate (a screw may start up to a turn higher up its thread for that), lifts it and brings it down
 * onto its mate in a straight line. A screw then turns the part
 * down its thread in straight moves of at most `max_turn_step_rad`, letting go and turning the
 * wrist back half a turn whenever the wrist nears its limit; a twist turns it in place. The
 * fingers let go, the arm rises and the part's pose is checked against its goal. The fingers may
 * touch the part, and the stage's contacts are allowed, only while the stage runs; the part may
 * touch what it rests on until it is lifted.
 *
 * @note Every call blocks until done and must come from a thread that no executor of the node
 * runs on; the services' answers come on the callback group given at construction.
 */
class StageRunner
{
public:
    /**
     * @brief Called with each phase a stage enters.
     */
    using Phase = std::function<void(const char *)>;

    /**
     * @brief Returns true once the stage is to stop.
     */
    using Cancelled = std::function<bool()>;

    /**
     * @brief Offers the stages of a task.
     *
     * @param[in] node The node that calls task_scene's services.
     * @param[in] group A callback group that answers the services while a stage runs.
     * @param[in] scene MoveIt's planning scene, where the parts are.
     * @param[in] stages The task's stages.
     * @param[in] settings Services, gripper, speeds and tolerances.
     */
    StageRunner(
        rclcpp::Node & node,
        const rclcpp::CallbackGroup::SharedPtr & group,
        std::shared_ptr<const SceneClient> scene,
        std::vector<Stage> stages,
        StageSettings settings);

    /**
     * @brief Runs one stage.
     *
     * @param[in] motion The arm and the gripper.
     * @param[in] name The stage's name.
     * @param[in] phase Told each phase the stage enters.
     * @param[in] cancelled Asked between moves whether to stop.
     *
     * @return How far the part ended from its goal, metres, or why the stage failed.
     */
    [[nodiscard]] std::expected<double, std::string> run(
        ArmMotion & motion,
        const std::string & name,
        const Phase & phase,
        const Cancelled & cancelled);

private:
    using Pairs = std::vector<std::pair<std::string, std::string>>;
    using Status = std::expected<void, std::string>;

    // Contacts allowed while a stage runs, forbidden again when it ends however it ends.
    class AllowedContacts
    {
    public:
        explicit AllowedContacts(StageRunner & runner);
        ~AllowedContacts();
        AllowedContacts(const AllowedContacts &) = delete;
        AllowedContacts & operator=(const AllowedContacts &) = delete;
        AllowedContacts(AllowedContacts &&) = delete;
        AllowedContacts & operator=(AllowedContacts &&) = delete;
        Status add(const Pairs & pairs);
        Status remove(const Pairs & pairs);

    private:
        StageRunner & runner_;
        Pairs pairs_;
    };

    struct Grasp
    {
        Eigen::Isometry3d in_part;
        JointValues pregrasp;
        // Where on the path the motion starts, as partOnMate's share: above 1 a screw starts
        // higher up the thread, which turns the part to where the wrist reaches it.
        double start_left{1.0};
    };

    [[nodiscard]] std::expected<Grasp, std::string> pick(
        ArmMotion & motion,
        const Stage & stage,
        const Eigen::Isometry3d & part,
        const Eigen::Isometry3d & mate,
        const Phase & phase,
        const Cancelled & cancelled) const;
    [[nodiscard]] Status place(
        ArmMotion & motion,
        const Stage & stage,
        const Eigen::Isometry3d & mate,
        const Grasp & grasp,
        AllowedContacts & allowed,
        const Phase & phase,
        const Cancelled & cancelled) const;
    [[nodiscard]] std::expected<double, std::string> check(const Stage & stage) const;
    [[nodiscard]] std::expected<Grasp, std::string> chooseGrasp(
        ArmMotion & motion,
        const Stage & stage,
        const Eigen::Isometry3d & part,
        const Eigen::Isometry3d & mate) const;
    [[nodiscard]] Status turn(
        ArmMotion & motion,
        const Stage & stage,
        const Eigen::Isometry3d & mate,
        const Eigen::Isometry3d & grasp,
        double from,
        double to) const;
    [[nodiscard]] Status screw(
        ArmMotion & motion,
        const Stage & stage,
        const Eigen::Isometry3d & mate,
        Eigen::Isometry3d grasp,
        double from,
        const Phase & phase,
        const Cancelled & cancelled) const;
    [[nodiscard]] Status close(ArmMotion & motion) const;
    [[nodiscard]] Status allow(const Pairs & pairs, bool allowed) const;
    [[nodiscard]] Status attach(const std::string & part, bool attach) const;

    std::shared_ptr<const SceneClient> scene_;
    std::vector<Stage> stages_;
    StageSettings settings_;
    rclcpp::Logger logger_;
    rclcpp::Client<piper_msgs::srv::AttachPart>::SharedPtr attach_client_;
    rclcpp::Client<piper_msgs::srv::AllowContacts>::SharedPtr allow_client_;
};

}  // namespace piper_skills
