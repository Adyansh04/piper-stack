#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <fstream>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <opencv2/core.hpp>
#include <piper_msgs/action/find_objects.hpp>
#include <piper_msgs/action/move_arm_to_named_pose.hpp>
#include <piper_msgs/action/pick_object.hpp>
#include <piper_msgs/action/place_object.hpp>
#include <piper_msgs/action/run_stage.hpp>
#include <piper_msgs/action/view_workspace.hpp>
#include <piper_msgs/msg/detected_object.hpp>
#include <piper_msgs/srv/compute_grasps.hpp>
#include <piper_msgs/srv/place_target.hpp>
#include <piper_msgs/srv/segment_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "piper_skills/arm_motion.hpp"
#include "piper_skills/camera_capture.hpp"
#include "piper_skills/grasp_selection.hpp"
#include "piper_skills/object_cloud.hpp"
#include "piper_skills/place_geometry.hpp"
#include "piper_skills/place_target.hpp"
#include "piper_skills/scene_client.hpp"
#include "piper_skills/srdf_states.hpp"
#include "piper_skills/stage_runner.hpp"
#include "piper_skills/task_stages.hpp"
#include "piper_skills/view_planner.hpp"

namespace piper_skills
{

namespace
{

using Status = std::expected<void, std::string>;

double degToRad(double deg)
{
    return deg * std::numbers::pi / 180.0;
}

std::chrono::milliseconds msOf(double s)
{
    return std::chrono::milliseconds(static_cast<std::int64_t>(s * 1000.0));
}

geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d & pose, const std::string & frame)
{
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame;
    out.pose = tf2::toMsg(pose);
    return out;
}

Eigen::Isometry3d turnedAboutApproach(const Eigen::Isometry3d & tcp)
{
    // The fingers are alike: half a turn about the approach is the same grasp, the wrist turned.
    return tcp * Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ());
}

/**
 * @brief An object found on the table, with the points it was seen as.
 */
struct Found
{
    piper_msgs::msg::DetectedObject object;
    std::vector<Eigen::Vector3d> points;
};

/**
 * @brief What the skills remember of the object in the fingers, for placing it.
 */
struct Held
{
    std::string label;
    Eigen::Matrix3d tcp_rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d centre_to_tcp{Eigen::Vector3d::Zero()};
    double tcp_above_bottom_m{0.0};
    double radius_m{0.0};
    // The yaw of the object's long axis on the table at the pick, radians.
    double long_yaw_rad{0.0};
    Eigen::Vector3d size_m{Eigen::Vector3d::Zero()};
};

constexpr const char * kHeldId = "held_object";
constexpr const char * kTableId = "table";

}  // namespace

/**
 * @class SkillsNode
 * @brief The Piper's manipulation skills on one table, as actions.
 * @details Offers `move_arm_to_named_pose`, `view_workspace`, `find_objects`, `pick_object`,
 * `place_object` and, with a task (`task.file`), `run_stage` (piper_msgs actions) and the
 * `place_target` service. One skill runs at a time: a
 * goal that comes while one runs is rejected. A skill runs on a thread of its own and checks for a
 * cancel between its steps; a cancel also stops the arm's move. Objects are found by name through
 * `segment_objects` (SAM 3.1) and grasped with `compute_grasps` (GraspGenX).
 */
class SkillsNode : public rclcpp::Node
{
public:
    /**
     * @brief Reads the parameters and offers the actions and the service.
     *
     * @param[in] options Node options from the component container.
     *
     * @throws std::invalid_argument If a parameter is out of range or the SRDF has no arm states.
     */
    explicit SkillsNode(const rclcpp::NodeOptions & options)
        : rclcpp::Node("piper_skills", options)
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_, *this)
    {
        readParameters();
        camera_ = std::make_unique<CameraCapture>(*this, camera_settings_, tf_buffer_);
        clients_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        segment_client_ = create_client<piper_msgs::srv::SegmentObjects>(
            segment_service_, rclcpp::ServicesQoS(), clients_group_);
        grasp_client_ = create_client<piper_msgs::srv::ComputeGrasps>(
            grasp_service_, rclcpp::ServicesQoS(), clients_group_);
        scene_ =
            std::make_shared<SceneClient>(*this, clients_group_, motion_settings_.service_wait);
        if (!task_stages_.empty())
        {
            stage_runner_ = std::make_unique<StageRunner>(
                *this, clients_group_, scene_, std::move(task_stages_), stage_settings_);
        }
        offerSkills();
        RCLCPP_INFO(
            get_logger(),
            "Skills ready: table top at z %.3f m, %zu views, states %s",
            top_.center.z(),
            views_.size(),
            stateNames().c_str());
    }

    ~SkillsNode() override
    {
        cancel_.store(true);
        if (motion_)
        {
            motion_->stop();
        }
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    SkillsNode(const SkillsNode &) = delete;
    SkillsNode & operator=(const SkillsNode &) = delete;
    SkillsNode(SkillsNode &&) = delete;
    SkillsNode & operator=(SkillsNode &&) = delete;

private:
    // ------------------------------------------------------------------------------------------
    // Parameters

    template <typename T>
    T param(const std::string & name, const T & fallback)
    {
        return declare_parameter<T>(name, fallback);
    }

    void readParameters()
    {
        planning_frame_ = param<std::string>("planning_frame", "world");
        camera_frame_ = param<std::string>("camera_frame", "wrist_camera_color_optical_frame");
        const auto srdf = param<std::string>("robot_description_semantic", "");
        motion_settings_.group = param<std::string>("arm_group", "arm");
        states_ = namedStates(srdf, motion_settings_.group);
        if (states_.empty())
        {
            throw std::invalid_argument(
                "robot_description_semantic has no named states of the arm group");
        }
        ready_state_ = param<std::string>("ready_state", "ready");
        if (!states_.contains(ready_state_))
        {
            throw std::invalid_argument(
                std::format("ready_state '{}' is not in the SRDF", ready_state_));
        }

        motion_settings_.tcp_link = param<std::string>("tcp_link", "tcp");
        motion_settings_.planning_frame = planning_frame_;
        motion_settings_.velocity_scale = param<double>("motion.velocity_scale", 0.3);
        motion_settings_.acceleration_scale = param<double>("motion.acceleration_scale", 0.3);
        motion_settings_.planning_time_s = param<double>("motion.planning_time_s", 5.0);
        motion_settings_.attempts = static_cast<int>(param<std::int64_t>("motion.attempts", 3));
        motion_settings_.ik_timeout = msOf(param<double>("motion.ik_timeout_s", 1.0));
        motion_settings_.service_wait = msOf(param<double>("motion.service_wait_s", 3.0));
        motion_settings_.gripper_action =
            param<std::string>("gripper.action", "/gripper_controller/gripper_cmd");
        motion_settings_.gripper_joint = param<std::string>("gripper.joint", "joint7");
        motion_settings_.gripper_timeout = msOf(param<double>("gripper.timeout_s", 15.0));
        gripper_open_m_ = param<double>("gripper.open_m", 0.035);
        gripper_closed_m_ = param<double>("gripper.closed_m", 0.0);
        gripper_effort_ = param<double>("gripper.effort", 1.0);
        held_min_m_ = param<double>("gripper.held_min_m", 0.002);
        width_tolerance_m_ = param<double>("gripper.width_tolerance_m", 0.02);
        touch_links_ = param<std::vector<std::string>>(
            "gripper.touch_links", {"gripper_base", "link6", "link7", "link8"});
        held_padding_m_ = param<double>("gripper.held_padding_m", 0.005);

        camera_settings_.colour_topic =
            param<std::string>("camera.colour_topic", "/wrist_camera/color/image_raw");
        camera_settings_.depth_topic =
            param<std::string>("camera.depth_topic", "/wrist_camera/depth/image_raw");
        camera_settings_.info_topic =
            param<std::string>("camera.info_topic", "/wrist_camera/color/camera_info");
        camera_settings_.planning_frame = planning_frame_;
        camera_settings_.settle = msOf(param<double>("camera.settle_s", 0.5));
        camera_settings_.timeout = msOf(param<double>("camera.timeout_s", 5.0));
        camera_settings_.pair_window = msOf(param<double>("camera.pair_window_s", 0.05));
        camera_settings_.tf_timeout = msOf(param<double>("camera.tf_timeout_s", 0.5));

        table_centre_ = xyParam("table.centre_m", {0.3, 0.0});
        table_size_ = xyParam("table.size_m", {0.8, 1.2});
        table_thickness_m_ = param<double>("table.thickness_m", 0.05);
        const auto top_z = param<double>("table.top_z_m", 0.0);
        const Eigen::Vector2d work_centre = xyParam("workspace.centre_m", {0.3, 0.0});
        const Eigen::Vector2d work_size = xyParam("workspace.size_m", {0.35, 0.6});
        top_.center = Eigen::Vector3d(work_centre.x(), work_centre.y(), top_z);
        top_.forward = Eigen::Vector3d::UnitX();
        top_.lateral = Eigen::Vector3d::UnitY();
        top_.half_depth_m = work_size.x() / 2.0;
        top_.half_width_m = work_size.y() / 2.0;
        axes_.origin = top_.center;
        axes_.x_axis = -Eigen::Vector3d::UnitY();
        axes_.y_axis = Eigen::Vector3d::UnitX();
        axes_.half_x_m = top_.half_width_m;
        axes_.half_y_m = top_.half_depth_m;

        view_.hfov_rad = degToRad(param<double>("view.hfov_deg", 77.1));
        view_.vfov_rad = degToRad(param<double>("view.vfov_deg", 61.7));
        view_.image_margin = param<double>("view.image_margin", 0.05);
        view_.occluded_bottom = param<double>("view.occluded_bottom", 0.0);
        const auto names =
            param<std::vector<std::string>>("view.names", {"full", "middle", "left", "right"});
        const auto states = param<std::vector<std::string>>(
            "view.states", {"ready", "ready", "view_left", "view_right"});
        if (names.size() != states.size())
        {
            throw std::invalid_argument("view.names and view.states differ in length");
        }
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            if (!states_.contains(states[i]))
            {
                throw std::invalid_argument(
                    std::format(
                        "view '{}' names state '{}', which the SRDF lacks", names[i], states[i]));
            }
            const auto region = placeArea(top_, names[i] == "full" ? "any" : names[i]);
            if (!region)
            {
                throw std::invalid_argument(
                    std::format("view '{}' is not full, left, middle or right", names[i]));
            }
            views_.push_back(TopView{.name = names[i], .region = *region, .candidates = {}});
            view_states_[names[i]] = states[i];
        }
        preferred_views_ = names;

        segment_service_ = param<std::string>("perception.segment_service", "/segment_objects");
        grasp_service_ = param<std::string>("perception.grasp_service", "/compute_grasps");
        segment_timeout_ = msOf(param<double>("perception.segment_timeout_s", 60.0));
        grasp_timeout_ = msOf(param<double>("perception.grasp_timeout_s", 60.0));
        default_min_score_ = param<double>("perception.min_score", 0.5);

        min_points_ = static_cast<std::size_t>(param<std::int64_t>("cloud.min_points", 50));
        cloud_.min_depth_m = param<double>("cloud.min_depth_m", 0.1);
        cloud_.max_depth_m = param<double>("cloud.max_depth_m", 1.0);
        cloud_.erode_px = static_cast<int>(param<std::int64_t>("cloud.erode_px", 2));
        cloud_.depth_band_m = param<double>("cloud.depth_band_m", 0.1);
        above_top_m_ = param<double>("cloud.above_top_m", 0.006);
        cluster_gap_m_ = param<double>("cloud.cluster_gap_m", 0.02);

        auto & g = grasp_filter_;
        g.max_approach_angle_rad = degToRad(param<double>("grasp.max_approach_angle_deg", 100.0));
        g.top_max_angle_rad = degToRad(param<double>("grasp.top_max_angle_deg", 50.0));
        g.max_closing_tilt_rad = degToRad(param<double>("grasp.max_closing_tilt_deg", 30.0));
        g.pads.width_m = param<double>("grasp.pad_width_m", 0.02);
        g.pads.length_m = param<double>("grasp.pad_length_m", 0.023);
        g.pads.open_gap_m = 2.0 * gripper_open_m_;
        g.pads.strips = static_cast<int>(param<std::int64_t>("grasp.contact_strips", 4));
        g.finger_thickness_m = param<double>("grasp.finger_thickness_m", 0.008);
        g.min_width_m = param<double>("grasp.min_width_m", 0.005);
        g.max_width_m = param<double>("grasp.max_width_m", 0.065);
        g.top_clearance_m = param<double>("grasp.top_clearance_m", 0.005);
        g.min_depth_m = param<double>("grasp.min_depth_m", 0.01);
        g.min_depth_share = param<double>("grasp.min_depth_share", 0.3);
        g.max_depth_m = param<double>("grasp.max_depth_m", 0.06);
        g.side_max_offset_share = param<double>("grasp.side_max_offset_share", 0.25);
        g.upright_weight = param<double>("grasp.upright_weight", 0.5);
        g.depth_weight = param<double>("grasp.depth_weight", 10.0);
        g.side_depth_weight = param<double>("grasp.side_depth_weight", 40.0);
        g.width_weight = param<double>("grasp.width_weight", 20.0);
        g.contact_weight = param<double>("grasp.contact_weight", 50.0);

        pregrasp_m_ = param<double>("pick.pregrasp_m", 0.06);
        lift_m_ = param<double>("pick.lift_m", 0.06);
        max_candidates_ = static_cast<std::size_t>(param<std::int64_t>("pick.max_candidates", 15));
        pick_attempts_ = static_cast<int>(param<std::int64_t>("pick.attempts", 3));

        release_height_m_ = param<double>("place.release_height_m", 0.01);
        place_approach_m_ = param<double>("place.approach_m", 0.06);
        clear_above_m_ = param<double>("place.clear_above_m", 0.05);
        place_tolerance_m_ = param<double>("place.tolerance_m", 0.05);
        free_.free_radius_m = param<double>("place.free_radius_m", 0.06);
        free_.grid_step_m = param<double>("place.grid_step_m", 0.01);
        free_.near_weight = param<double>("place.near_weight", 0.5);
        free_.max_reach_m = param<double>("place.max_reach_m", 0.33);
        free_.min_camera_distance_m = param<double>("place.min_camera_distance_m", 0.2);
        free_.wrist_above_m = param<double>("place.wrist_above_m", 0.25);
        free_.wrist_radius_m = param<double>("place.wrist_radius_m", 0.08);
        free_.min_depth_m = cloud_.min_depth_m;
        free_.max_depth_m = cloud_.max_depth_m;

        const auto task_file = param<std::string>("task.file", "");
        auto & t = stage_settings_;
        t.scene_node = param<std::string>("task.scene_node", "/task_scene");
        t.touch_links = touch_links_;
        t.closed_m = gripper_closed_m_;
        t.effort = gripper_effort_;
        t.held_min_m = param<double>("task.held_min_m", 0.0003);
        t.squeeze_m = param<double>("task.squeeze_m", 0.003);
        t.pregrasp_m = pregrasp_m_;
        t.lift_m = lift_m_;
        t.insert_velocity_scale = param<double>("task.insert_velocity_scale", 0.1);
        t.turn_velocity_scale = param<double>("task.turn_velocity_scale", 0.6);
        t.max_turn_step_rad = degToRad(param<double>("task.max_turn_step_deg", 90.0));
        t.wrist_joint = param<std::string>("task.wrist_joint", "joint6");
        t.wrist_margin_rad = param<double>("task.wrist_margin_rad", 0.15);
        t.scene_settle = msOf(param<double>("task.scene_settle_s", 0.3));
        t.screw_step_rad = degToRad(param<double>("task.screw_step_deg", 45.0));
        t.screw_preload_m = param<double>("task.screw_preload_m", 0.0005);
        t.screw_seated_m = param<double>("task.screw_seated_m", 0.0005);
        t.screw_search_turns = param<double>("task.screw_search_turns", 1.5);
        t.settle = msOf(param<double>("task.settle_s", 1.0));
        t.goal_tolerance_m = param<double>("task.goal_tolerance_m", 0.003);
        t.goal_tolerance_rad = degToRad(param<double>("task.goal_tolerance_deg", 20.0));
        t.service_timeout = motion_settings_.service_wait;
        if (!task_file.empty())
        {
            const std::ifstream file(task_file);
            if (!file)
            {
                throw std::invalid_argument(std::format("task.file {} does not open", task_file));
            }
            std::stringstream text;
            text << file.rdbuf();
            task_stages_ = parseStages(text.str());
        }
    }

    Eigen::Vector2d xyParam(const std::string & name, const std::vector<double> & fallback)
    {
        const auto value = param<std::vector<double>>(name, fallback);
        if (value.size() != 2)
        {
            throw std::invalid_argument(std::format("{} is [x, y], metres", name));
        }
        return {value[0], value[1]};
    }

    std::string stateNames() const
    {
        std::string out;
        for (const auto & [name, joints] : states_)
        {
            out += out.empty() ? name : ", " + name;
        }
        return out;
    }

    // ------------------------------------------------------------------------------------------
    // Serving: one skill at a time on a worker thread

    template <typename Action>
    using Handle = rclcpp_action::ServerGoalHandle<Action>;

    template <typename Action>
    using Body =
        std::function<Status(const std::shared_ptr<Handle<Action>> &, typename Action::Result &)>;

    template <typename Action>
    typename rclcpp_action::Server<Action>::SharedPtr serve(
        const std::string & name, const Body<Action> & body)
    {
        return rclcpp_action::create_server<Action>(
            this,
            name,
            [this, name](
                const rclcpp_action::GoalUUID &,
                const std::shared_ptr<const typename Action::Goal> &) {
                if (busy_.exchange(true))
                {
                    RCLCPP_WARN(get_logger(), "%s refused: a skill is running", name.c_str());
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<Handle<Action>> &) {
                cancel_.store(true);
                const std::scoped_lock lock(motion_mutex_);
                if (motion_)
                {
                    motion_->stop();
                }
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this, name, body](const std::shared_ptr<Handle<Action>> & goal) {
                // The last worker has ended (busy_ was clear), so this join returns at once.
                worker_ = std::jthread([this, name, body, goal] { run<Action>(name, body, goal); });
            },
            rcl_action_server_get_default_options(),
            actions_group_);
    }

    template <typename Action>
    void run(
        const std::string & name,
        const Body<Action> & body,
        const std::shared_ptr<Handle<Action>> & goal)
    {
        auto result = std::make_shared<typename Action::Result>();
        cancel_.store(false);
        Status status;
        try
        {
            if (!motion_)
            {
                // Made outside the lock: connecting waits for move_group, and a cancel must not.
                auto motion = std::make_unique<ArmMotion>(shared_from_this(), motion_settings_);
                const std::scoped_lock lock(motion_mutex_);
                motion_ = std::move(motion);
                addTable();
            }
            motion_->clearStop();
            status = body(goal, *result);
        }
        catch (const std::exception & error)
        {
            status = std::unexpected(std::format("{} failed: {}", name, error.what()));
        }
        result->success = status.has_value();
        result->message = status ? std::string("done") : status.error();
        if (goal->is_canceling())
        {
            result->success = false;
            goal->canceled(result);
        }
        else if (status)
        {
            goal->succeed(result);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "%s: %s", name.c_str(), status.error().c_str());
            goal->abort(result);
        }
        busy_.store(false);
    }

    void offerSkills()
    {
        actions_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        named_pose_server_ = serve<piper_msgs::action::MoveArmToNamedPose>(
            "move_arm_to_named_pose",
            [this](const auto & goal, auto &) { return runNamedPose(*goal->get_goal()); });
        view_server_ = serve<piper_msgs::action::ViewWorkspace>(
            "view_workspace", [this](const auto & goal, auto & result) {
                return runView(goal->get_goal()->view, result.camera_pose);
            });
        find_server_ = serve<piper_msgs::action::FindObjects>(
            "find_objects",
            [this](const auto & goal, auto & result) { return runFind(goal, result); });
        pick_server_ = serve<piper_msgs::action::PickObject>(
            "pick_object",
            [this](const auto & goal, auto & result) { return runPick(goal, result); });
        place_server_ = serve<piper_msgs::action::PlaceObject>(
            "place_object",
            [this](const auto & goal, auto & result) { return runPlace(goal, result); });
        stage_server_ = serve<piper_msgs::action::RunStage>(
            "run_stage",
            [this](const auto & goal, auto & result) { return runStage(goal, result); });
        place_target_service_ = create_service<piper_msgs::srv::PlaceTarget>(
            "place_target",
            [this](
                const std::shared_ptr<piper_msgs::srv::PlaceTarget::Request> & request,
                const std::shared_ptr<piper_msgs::srv::PlaceTarget::Response> & response) {
                answerPlaceTarget(*request, *response);
            },
            rclcpp::ServicesQoS(),
            clients_group_);
    }

    Status checkCancel() const
    {
        if (cancel_.load())
        {
            return std::unexpected("cancelled");
        }
        return {};
    }

    // ------------------------------------------------------------------------------------------
    // Scene

    void addTable()
    {
        moveit_msgs::msg::CollisionObject table;
        table.id = kTableId;
        table.header.frame_id = planning_frame_;
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {table_size_.x(), table_size_.y(), table_thickness_m_};
        table.primitives.push_back(box);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        // Its top a millimetre below the real one: the arm's base stands on it.
        pose.translation() = Eigen::Vector3d(
            table_centre_.x(),
            table_centre_.y(),
            top_.center.z() - table_thickness_m_ / 2.0 - 0.001);
        table.primitive_poses.push_back(tf2::toMsg(pose));
        table.operation = moveit_msgs::msg::CollisionObject::ADD;
        moveit_msgs::msg::PlanningScene diff;
        diff.world.collision_objects.push_back(std::move(table));
        warnIfFailed(scene_->apply(std::move(diff)), "Adding the table");
    }

    void attachHeld(const Eigen::Vector3d & centre_in_tcp, const Eigen::Vector3d & size_m)
    {
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = motion_settings_.tcp_link;
        attached.touch_links = touch_links_;
        auto & object = attached.object;
        object.id = kHeldId;
        object.header.frame_id = motion_settings_.tcp_link;
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {
            size_m.x() + held_padding_m_,
            size_m.y() + held_padding_m_,
            size_m.z() + held_padding_m_};
        object.primitives.push_back(box);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = centre_in_tcp;
        object.primitive_poses.push_back(tf2::toMsg(pose));
        object.operation = moveit_msgs::msg::CollisionObject::ADD;
        moveit_msgs::msg::PlanningScene diff;
        diff.robot_state.is_diff = true;
        diff.robot_state.attached_collision_objects.push_back(std::move(attached));
        warnIfFailed(scene_->apply(std::move(diff)), "Attaching the held object");
    }

    void detachHeld()
    {
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = motion_settings_.tcp_link;
        attached.object.id = kHeldId;
        attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        // Detached, the object would stay in the world where it was let go: remove it there too.
        moveit_msgs::msg::CollisionObject gone;
        gone.id = kHeldId;
        gone.header.frame_id = planning_frame_;
        gone.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        moveit_msgs::msg::PlanningScene diff;
        diff.robot_state.is_diff = true;
        diff.robot_state.attached_collision_objects.push_back(std::move(attached));
        diff.world.collision_objects.push_back(std::move(gone));
        warnIfFailed(scene_->apply(std::move(diff)), "Letting go of the held object");
        held_.reset();
    }

    // ------------------------------------------------------------------------------------------
    // Motion helpers

    Status moveTo(const std::string & state, double velocity_scale = 0.0)
    {
        const auto found = states_.find(state);
        if (found == states_.end())
        {
            return std::unexpected(
                std::format("no named pose '{}' (known: {})", state, stateNames()));
        }
        return motion_->toJoints(found->second, velocity_scale);
    }

    /// The camera's pose now, in the planning frame.
    std::expected<geometry_msgs::msg::PoseStamped, std::string> cameraPose()
    {
        try
        {
            geometry_msgs::msg::PoseStamped out;
            const auto tf = tf_buffer_.lookupTransform(
                planning_frame_, camera_frame_, tf2::TimePointZero, std::chrono::milliseconds(500));
            out.header = tf.header;
            out.pose.position.x = tf.transform.translation.x;
            out.pose.position.y = tf.transform.translation.y;
            out.pose.position.z = tf.transform.translation.z;
            out.pose.orientation = tf.transform.rotation;
            return out;
        }
        catch (const tf2::TransformException & error)
        {
            return std::unexpected(
                std::format(
                    "no transform {} <- {}: {}", planning_frame_, camera_frame_, error.what()));
        }
    }

    /// The view whose region's centre is nearest an area's.
    std::string viewFor(const TopRegion & area) const
    {
        const auto nearest = std::ranges::min_element(
            views_, {}, [&](const TopView & v) { return (v.region.center - area.center).norm(); });
        return nearest == views_.end() ? std::string("full") : nearest->name;
    }

    // ------------------------------------------------------------------------------------------
    // Perception

    std::expected<std::vector<Found>, std::string> findIn(
        const CapturedFrame & frame, const std::string & prompt, double min_score)
    {
        if (!segment_client_->wait_for_service(motion_settings_.service_wait))
        {
            return std::unexpected(std::format("{} is not offered", segment_service_));
        }
        auto request = std::make_shared<piper_msgs::srv::SegmentObjects::Request>();
        request->image = *frame.colour;
        request->prompts = {prompt};
        request->min_score = static_cast<float>(min_score);
        auto future = segment_client_->async_send_request(request);
        if (future.wait_for(segment_timeout_) != std::future_status::ready)
        {
            segment_client_->remove_pending_request(future);
            return std::unexpected("segment_objects did not answer");
        }
        const auto response = future.get();
        if (!response->success)
        {
            return std::unexpected(std::format("segmentation failed: {}", response->message));
        }
        std::vector<Found> found;
        for (const auto & segment : response->segments)
        {
            const cv::Mat full(
                static_cast<int>(frame.colour->height),
                static_cast<int>(frame.colour->width),
                CV_8UC1,
                cv::Scalar(0));
            const cv::Rect roi(
                static_cast<int>(segment.roi.x_offset),
                static_cast<int>(segment.roi.y_offset),
                static_cast<int>(segment.roi.width),
                static_cast<int>(segment.roi.height));
            try
            {
                cv_bridge::toCvCopy(segment.mask, "mono8")->image.copyTo(full(roi));
            }
            catch (const std::exception & error)
            {
                RCLCPP_WARN(
                    get_logger(), "Skipping a mask of '%s': %s", prompt.c_str(), error.what());
                continue;
            }
            Found f;
            for (const Eigen::Vector3f & p : maskPoints(frame.depth_m, full, frame.camera, cloud_))
            {
                f.points.push_back(frame.base_to_camera * p.cast<double>());
            }
            // The mask can spill onto the table and onto a neighbour: only the object's own points.
            f.points = ownPoints(f.points, top_.center.z(), above_top_m_, cluster_gap_m_);
            if (f.points.size() < min_points_)
            {
                continue;
            }
            const ObjectShape shape = describe(f.points);
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear() = shape.axes;
            pose.translation() = shape.centroid;
            auto & o = f.object;
            o.id = std::format("{}_{}", prompt, found.size());
            std::ranges::replace(o.id, ' ', '_');
            o.label = prompt;
            o.score = segment.score;
            o.pose = stamped(pose, planning_frame_);
            o.pose.header.stamp = frame.colour->header.stamp;
            o.size.x = shape.size.x();
            o.size.y = shape.size.y();
            o.size.z = shape.size.z();
            o.point_count = static_cast<std::uint32_t>(f.points.size());
            RCLCPP_INFO(
                get_logger(),
                "Found '%s' (%.2f) at (%.3f, %.3f, %.3f) m, %u points",
                prompt.c_str(),
                static_cast<double>(o.score),
                shape.centroid.x(),
                shape.centroid.y(),
                shape.centroid.z(),
                o.point_count);
            found.push_back(std::move(f));
        }
        // Off the table: not an object on the workspace (the table's axes are the base's x and y).
        std::erase_if(found, [this](const Found & f) {
            const auto & p = f.object.pose.pose.position;
            return std::abs(p.x - top_.center.x()) > top_.half_depth_m
                   || std::abs(p.y - top_.center.y()) > top_.half_width_m;
        });
        std::ranges::sort(found, [](const Found & a, const Found & b) {
            return a.object.score > b.object.score;
        });
        return found;
    }

    /// Looks through the views in order and returns the first view's finds.
    std::expected<std::pair<std::string, std::vector<Found>>, std::string> search(
        const std::string & prompt,
        const std::vector<std::string> & asked,
        double min_score,
        const std::function<void(const std::string &)> & on_view)
    {
        std::string why = "nothing found";
        for (const std::string & view : viewOrder(views_, asked, preferred_views_))
        {
            if (const auto stop = checkCancel(); !stop)
            {
                return std::unexpected(stop.error());
            }
            on_view(view);
            if (const auto moved = runView(view, nullptr); !moved)
            {
                why = moved.error();
                continue;
            }
            const auto frame = camera_->capture(cancel_);
            if (!frame)
            {
                return std::unexpected(frame.error());
            }
            auto found = findIn(*frame, prompt, min_score);
            if (!found)
            {
                return std::unexpected(found.error());
            }
            if (!found->empty())
            {
                return std::make_pair(view, std::move(*found));
            }
        }
        return std::make_pair(std::string(), std::vector<Found>{});
    }

    std::expected<std::vector<GraspCandidate>, std::string> requestGrasps(const Found & object)
    {
        if (!grasp_client_->wait_for_service(motion_settings_.service_wait))
        {
            return std::unexpected(std::format("{} is not offered", grasp_service_));
        }
        auto request = std::make_shared<piper_msgs::srv::ComputeGrasps::Request>();
        auto & cloud = request->object_cloud;
        cloud.header.frame_id = planning_frame_;
        cloud.header.stamp = now();
        for (std::uint32_t i = 0; i < 3; ++i)
        {
            sensor_msgs::msg::PointField field;
            field.name = std::string(1, "xyz"[i]);
            field.offset = 4 * i;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            cloud.fields.push_back(field);
        }
        cloud.height = 1;
        cloud.width = static_cast<std::uint32_t>(object.points.size());
        cloud.point_step = 12;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.is_dense = true;
        cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        for (const Eigen::Vector3d & p : object.points)
        {
            *x = static_cast<float>(p.x());
            *y = static_cast<float>(p.y());
            *z = static_cast<float>(p.z());
            ++x;
            ++y;
            ++z;
        }
        auto future = grasp_client_->async_send_request(request);
        if (future.wait_for(grasp_timeout_) != std::future_status::ready)
        {
            grasp_client_->remove_pending_request(future);
            return std::unexpected("compute_grasps did not answer");
        }
        const auto response = future.get();
        if (!response->success)
        {
            return std::unexpected(std::format("no grasps: {}", response->message));
        }
        std::vector<GraspCandidate> candidates;
        candidates.reserve(response->grasps.size());
        for (const auto & g : response->grasps)
        {
            GraspCandidate c;
            tf2::fromMsg(g.pose.pose, c.tcp);
            // Closing, the pads push the object onto the fingers' centre plane: centre on it.
            c.tcp.translation() +=
                c.tcp.linear().col(1) * closingOffset(object.points, c.tcp, grasp_filter_.pads);
            c.score = g.score;
            c.width_m = g.width_m;
            c.object_width_m = g.object_width_m;
            c.contact_spread_m = contactSpread(object.points, c.tcp, grasp_filter_.pads);
            c.depth_m = graspDepth(object.points, c.tcp, grasp_filter_.pads);
            candidates.push_back(c);
        }
        return candidates;
    }

    // ------------------------------------------------------------------------------------------
    // Skills

    Status runNamedPose(const piper_msgs::action::MoveArmToNamedPose::Goal & goal)
    {
        if (held_ && !holding())
        {
            detachHeld();
        }
        return moveTo(goal.name, goal.velocity_scale);
    }

    Status runView(const std::string & name, geometry_msgs::msg::PoseStamped * shown)
    {
        const std::string wanted = name.empty() ? std::string("full") : name;
        const auto state = view_states_.find(wanted);
        if (state == view_states_.end())
        {
            std::string known;
            for (const TopView & v : views_)
            {
                known += known.empty() ? v.name : ", " + v.name;
            }
            return std::unexpected(std::format("no view '{}' (known: {})", wanted, known));
        }
        if (const auto moved = moveTo(state->second); !moved)
        {
            return moved;
        }
        if (shown != nullptr)
        {
            const auto pose = cameraPose();
            if (!pose)
            {
                return std::unexpected(pose.error());
            }
            *shown = *pose;
        }
        return {};
    }

    Status runView(const std::string & name, geometry_msgs::msg::PoseStamped & shown)
    {
        return runView(name, &shown);
    }

    Status runFind(
        const std::shared_ptr<Handle<piper_msgs::action::FindObjects>> & goal,
        piper_msgs::action::FindObjects::Result & result)
    {
        const auto & g = *goal->get_goal();
        if (g.prompt.empty())
        {
            return std::unexpected("an empty prompt");
        }
        const double min_score = g.min_score > 0.0F ? g.min_score : default_min_score_;
        auto feedback = std::make_shared<piper_msgs::action::FindObjects::Feedback>();
        const auto found = search(g.prompt, g.views, min_score, [&](const std::string & view) {
            feedback->view = view;
            goal->publish_feedback(feedback);
        });
        if (!found)
        {
            return std::unexpected(found.error());
        }
        result.view = found->first;
        result.nothing_in_view = found->second.empty();
        for (const Found & f : found->second)
        {
            result.objects.push_back(f.object);
        }
        if (found->second.empty())
        {
            return std::unexpected(std::format("no '{}' on the table", g.prompt));
        }
        return {};
    }

    /// Whether the fingers still hold something: the gripper controller keeps closing them after a
    /// grasp, so an object that slipped out leaves them closed.
    bool holding()
    {
        const auto travel = motion_->fingerTravel();
        return travel && *travel > held_min_m_;
    }

    Status runPick(
        const std::shared_ptr<Handle<piper_msgs::action::PickObject>> & goal,
        piper_msgs::action::PickObject::Result & result)
    {
        const auto & g = *goal->get_goal();
        auto feedback = std::make_shared<piper_msgs::action::PickObject::Feedback>();
        const auto phase = [&](const char * name) {
            feedback->phase = name;
            goal->publish_feedback(feedback);
        };
        const auto approach = parseApproach(g.approach);
        if (!approach)
        {
            return std::unexpected(approach.error());
        }
        if (held_)
        {
            return std::unexpected(
                std::format("the gripper holds '{}': place it first", held_->label));
        }
        phase("finding");
        const double min_score = g.min_score > 0.0F ? g.min_score : default_min_score_;
        const auto found = search(g.prompt, {}, min_score, [](const std::string &) {});
        if (!found)
        {
            return std::unexpected(found.error());
        }
        if (found->second.empty())
        {
            return std::unexpected(std::format("no '{}' on the table", g.prompt));
        }
        const Found & object = found->second.front();
        double object_top_m = -1.0e9;
        for (const Eigen::Vector3d & p : object.points)
        {
            object_top_m = std::max(object_top_m, p.z());
        }

        phase("planning_grasps");
        const auto candidates = requestGrasps(object);
        if (!candidates)
        {
            return std::unexpected(candidates.error());
        }
        const GraspRanking ranking =
            rankGrasps(*candidates, grasp_filter_, top_.center.z(), object_top_m, *approach);
        if (ranking.order.empty())
        {
            return std::unexpected(
                std::format(
                    "none of {} grasps suits the gripper (too tilted {}, narrow {}, wide {}, low "
                    "{}, "
                    "shallow {}, deep {}, finger under {}, off centre {}, other approach {})",
                    candidates->size(),
                    ranking.too_tilted,
                    ranking.too_narrow,
                    ranking.too_wide,
                    ranking.too_low,
                    ranking.too_shallow,
                    ranking.too_deep,
                    ranking.finger_under,
                    ranking.off_centre,
                    ranking.other_approach));
        }
        return tryGrasps(*candidates, ranking, object, phase, result);
    }

    static std::expected<GraspApproach, std::string> parseApproach(const std::string & text)
    {
        if (text.empty() || text == "auto")
        {
            return GraspApproach::kAny;
        }
        if (text == "top")
        {
            return GraspApproach::kTop;
        }
        if (text == "side")
        {
            return GraspApproach::kSide;
        }
        return std::unexpected(std::format("approach '{}' is not auto, top or side", text));
    }

    /// The best reachable grasps in turn, each also turned half a turn, until one holds.
    Status tryGrasps(
        const std::vector<GraspCandidate> & candidates,
        const GraspRanking & ranking,
        const Found & object,
        const std::function<void(const char *)> & phase,
        piper_msgs::action::PickObject::Result & result)
    {
        std::string why = "no reachable grasp";
        int attempts = 0;
        for (std::size_t k = 0; k < std::min(ranking.order.size(), max_candidates_); ++k)
        {
            const GraspCandidate & candidate = candidates[ranking.order[k]];
            for (const Eigen::Isometry3d & tcp :
                 {candidate.tcp, turnedAboutApproach(candidate.tcp)})
            {
                if (const auto stop = checkCancel(); !stop)
                {
                    return stop;
                }
                const auto above = motion_->ik(backOff(tcp, pregrasp_m_));
                if (!above)
                {
                    why = above.error();
                    continue;
                }
                if (attempts++ >= pick_attempts_)
                {
                    return std::unexpected(
                        std::format("{} attempts failed: {}", pick_attempts_, why));
                }
                const Eigen::Vector3d at = tcp.translation();
                const Eigen::Vector3d approach = tcp.linear().col(2);
                RCLCPP_INFO(
                    get_logger(),
                    "Grasp %zu: TCP at (%.3f, %.3f, %.3f), approach (%.2f, %.2f, %.2f), depth %.1f "
                    "mm, opening %.1f mm on %.1f mm, score %.2f",
                    k,
                    at.x(),
                    at.y(),
                    at.z(),
                    approach.x(),
                    approach.y(),
                    approach.z(),
                    1000.0 * candidate.depth_m,
                    1000.0 * candidate.width_m,
                    1000.0 * candidate.object_width_m,
                    candidate.score);
                const auto grasped = graspAndLift(tcp, *above, candidate, object, phase);
                if (!grasped)
                {
                    why = grasped.error();
                    RCLCPP_WARN(get_logger(), "Grasp %zu failed: %s", k, why.c_str());
                    continue;
                }
                result.grasp.pose = stamped(tcp, planning_frame_);
                result.grasp.width_m = static_cast<float>(candidate.width_m);
                result.grasp.object_width_m = static_cast<float>(candidate.object_width_m);
                result.grasp.score = static_cast<float>(candidate.score);
                result.finger_width_m = 2.0 * *grasped;
                phase("retreating");
                return moveTo(ready_state_);
            }
        }
        return std::unexpected(why);
    }

    template <typename T>
    void warnIfFailed(const std::expected<T, std::string> & outcome, const char * what) const
    {
        if (!outcome)
        {
            RCLCPP_WARN(get_logger(), "%s failed: %s", what, outcome.error().c_str());
        }
    }

    /// One grasp: above, down, close, check, up. Returns the finger's travel on the object.
    std::expected<double, std::string> graspAndLift(
        const Eigen::Isometry3d & tcp,
        const JointValues & above,
        const GraspCandidate & candidate,
        const Found & object,
        const std::function<void(const char *)> & phase)
    {
        phase("moving_above");
        if (const auto opened = motion_->gripper(gripper_open_m_, gripper_effort_); !opened)
        {
            return std::unexpected(opened.error());
        }
        if (const auto moved = motion_->toJoints(above); !moved)
        {
            return std::unexpected(moved.error());
        }
        if (const auto stop = checkCancel(); !stop)
        {
            return std::unexpected(stop.error());
        }
        phase("descending");
        if (const auto down = motion_->linear(tcp); !down)
        {
            warnIfFailed(motion_->toJoints(above), "Backing up after a failed descent");
            return std::unexpected(down.error());
        }
        phase("closing");
        const auto finger = motion_->gripper(gripper_closed_m_, gripper_effort_);
        if (!finger)
        {
            return std::unexpected(finger.error());
        }
        const double opening_m = 2.0 * *finger;
        if (*finger <= held_min_m_
            || std::abs(opening_m - candidate.object_width_m) > width_tolerance_m_)
        {
            warnIfFailed(
                motion_->gripper(gripper_open_m_, gripper_effort_), "Opening after a miss");
            warnIfFailed(motion_->linear(backOff(tcp, pregrasp_m_)), "Backing off after a miss");
            return std::unexpected(
                std::format(
                    "the fingers stopped at {:.1f} mm, the object is {:.1f} mm wide",
                    1000.0 * opening_m,
                    1000.0 * candidate.object_width_m));
        }
        RCLCPP_INFO(
            get_logger(),
            "Closed at %.1f mm on the %.1f mm object",
            1000.0 * opening_m,
            1000.0 * candidate.object_width_m);
        // Held from here: what MoveIt plans next carries the object, its box padded.
        Eigen::Isometry3d object_pose;
        tf2::fromMsg(object.object.pose.pose, object_pose);
        const Eigen::Vector3d centre = boxCentre(object_pose, object.points);
        const Footprint footprint = footprintOf(object.points);
        Held held;
        held.label = object.object.label;
        held.tcp_rotation = tcp.linear();
        held.centre_to_tcp = tcp.translation() - centre;
        held.tcp_above_bottom_m = tcp.translation().z() - top_.center.z();
        held.radius_m = footprint.radius_m;
        held.long_yaw_rad = std::atan2(footprint.long_axis.y(), footprint.long_axis.x());
        held.size_m =
            Eigen::Vector3d(object.object.size.x, object.object.size.y, object.object.size.z);
        attachHeld(tcp.inverse() * centre, held.size_m);
        held_ = held;
        phase("lifting");
        Eigen::Isometry3d up = tcp;
        up.translation().z() += lift_m_;
        if (const auto lifted = motion_->linear(up); !lifted)
        {
            return std::unexpected(lifted.error());
        }
        const auto still = motion_->fingerTravel();
        if (!still || *still <= held_min_m_)
        {
            detachHeld();
            warnIfFailed(
                motion_->gripper(gripper_open_m_, gripper_effort_), "Opening after a slip");
            return std::unexpected(
                still
                    ? std::format(
                          "the object slipped out while lifting (the fingers closed to {:.1f} mm)",
                          2000.0 * *still)
                    : std::string("no current finger position after the lift"));
        }
        return *finger;
    }

    std::expected<Eigen::Vector3d, std::string> placeTargetPoint(
        PlaceMode mode,
        double x,
        double y,
        const std::optional<Eigen::Vector3d> & anchor,
        double inset_m) const
    {
        if (mode == PlaceMode::kFree)
        {
            return Eigen::Vector3d(top_.center);
        }
        const PlaceTarget target = placeTarget(axes_, mode, x, y, inset_m, anchor);
        if (!target.point)
        {
            return std::unexpected(target.why);
        }
        return *target.point;
    }

    Status runPlace(
        const std::shared_ptr<Handle<piper_msgs::action::PlaceObject>> & goal,
        piper_msgs::action::PlaceObject::Result & result)
    {
        const auto & g = *goal->get_goal();
        auto feedback = std::make_shared<piper_msgs::action::PlaceObject::Feedback>();
        const auto phase = [&](const char * name) {
            feedback->phase = name;
            goal->publish_feedback(feedback);
        };
        if (!held_)
        {
            return std::unexpected("the gripper holds nothing to place");
        }
        const Held held = *held_;
        phase("resolving_target");
        const auto area = placeRegion(g, held);
        if (!area)
        {
            return std::unexpected(area.error());
        }
        phase("viewing");
        const auto spot = freeSpotIn(*area, held);
        if (!spot)
        {
            return std::unexpected(spot.error());
        }
        phase("planning_place");
        std::vector<double> turns{
            0.0, std::numbers::pi, std::numbers::pi / 2.0, -std::numbers::pi / 2.0};
        if (g.use_yaw)
        {
            turns = turnsForYaw(
                held.long_yaw_rad,
                degToRad(g.yaw_deg),
                degToRad(g.yaw_tolerance_deg > 0.0 ? g.yaw_tolerance_deg : 10.0),
                degToRad(5.0));
        }
        return putDown(*spot, turns, held, phase, result);
    }

    /// The part of the table a place goal asks for: a region, or the area around its target.
    std::expected<TopRegion, std::string> placeRegion(
        const piper_msgs::action::PlaceObject::Goal & g, const Held & held)
    {
        const std::optional<PlaceMode> mode = parsePlaceMode(g.mode);
        if (!mode)
        {
            return std::unexpected(
                std::format("place mode '{}' is not free, relative, metric or anchor", g.mode));
        }
        if (*mode == PlaceMode::kFree)
        {
            const auto region = placeArea(top_, g.region.empty() ? std::string("any") : g.region);
            if (!region)
            {
                return std::unexpected(
                    std::format("region '{}' is not any, left, middle or right", g.region));
            }
            return *region;
        }
        std::optional<Eigen::Vector3d> anchor;
        if (*mode == PlaceMode::kAnchor && !g.anchor_pose.header.frame_id.empty())
        {
            const auto & p = g.anchor_pose.pose.position;
            anchor = Eigen::Vector3d(p.x, p.y, p.z);
        }
        else if (*mode == PlaceMode::kAnchor)
        {
            const auto found = search(g.anchor, {}, default_min_score_, [](const std::string &) {});
            if (!found || found->second.empty())
            {
                return std::unexpected(std::format("no anchor '{}' on the table", g.anchor));
            }
            const auto & p = found->second.front().object.pose.pose.position;
            anchor = Eigen::Vector3d(p.x, p.y, p.z);
        }
        const auto target = placeTargetPoint(*mode, g.x, g.y, anchor, held.radius_m);
        if (!target)
        {
            return std::unexpected(target.error());
        }
        const double tolerance_m = g.tolerance_m > 0.0 ? g.tolerance_m : place_tolerance_m_;
        return targetArea(top_, *target, tolerance_m, free_.free_radius_m);
    }

    /// Looks at an area and finds the free spot in it nearest its centre and the robot: from the
    /// area's own view first, then from the views of other states, since the gripper and what it
    /// holds hide the table under the camera.
    std::expected<Eigen::Vector3d, std::string> freeSpotIn(
        const TopRegion & area, const Held & held)
    {
        std::vector<std::string> order{viewFor(area)};
        for (const std::string & name : preferred_views_)
        {
            if (std::ranges::none_of(order, [&](const std::string & taken) {
                    return view_states_.at(taken) == view_states_.at(name);
                }))
            {
                order.push_back(name);
            }
        }
        std::string why;
        for (const std::string & view : order)
        {
            const auto spot = freeSpotFrom(view, area, held);
            if (!spot)
            {
                return std::unexpected(spot.error());
            }
            if (spot->spot)
            {
                return *spot->spot;
            }
            const FreeSpotCounts & c = spot->counts;
            why = std::format(
                "no free spot there ({} checked, {} unseen, {} taken, {} without depth, {} out of "
                "reach, from view {})",
                c.checked,
                c.unseen,
                c.taken,
                c.no_depth,
                c.out_of_reach,
                view);
            RCLCPP_INFO(get_logger(), "Place: %s", why.c_str());
        }
        return std::unexpected(why);
    }

    /// The free-spot search over an area from one view; an error when the arm or camera fails.
    std::expected<FreeSpotResult, std::string> freeSpotFrom(
        const std::string & view, const TopRegion & area, const Held & held)
    {
        if (const auto seen = runView(view, nullptr); !seen)
        {
            return std::unexpected(seen.error());
        }
        const auto frame = camera_->capture(cancel_);
        if (!frame)
        {
            return std::unexpected(frame.error());
        }
        FreeSpotOptions options = free_;
        options.free_radius_m = std::max(free_.free_radius_m, held.radius_m);
        // The gripper and what it holds hang in the image: they take no spot.
        options.tcp = motion_->tcpPose().translation();
        options.held_radius_m = held.radius_m + held_padding_m_;
        options.held_below_m = held.size_m.z() + held_padding_m_;
        return freeSpot(
            frame->depth_m, frame->camera, frame->base_to_camera, top_, area, view_, options);
    }

    /// Puts the held object down on a spot, trying the turns in order: above, down, open, up.
    Status putDown(
        const Eigen::Vector3d & spot,
        const std::vector<double> & turns,
        const Held & held,
        const std::function<void(const char *)> & phase,
        piper_msgs::action::PlaceObject::Result & result)
    {
        std::string why = "no reachable place";
        for (const double turn : turns)
        {
            if (const auto stop = checkCancel(); !stop)
            {
                return stop;
            }
            const Eigen::Isometry3d release = placeTcp(
                spot,
                turn,
                held.tcp_rotation,
                held.centre_to_tcp,
                held.tcp_above_bottom_m,
                release_height_m_);
            Eigen::Isometry3d above = release;
            above.translation().z() += place_approach_m_;
            const Eigen::Vector3d approach = release.linear().col(2);
            RCLCPP_INFO(
                get_logger(),
                "Place at (%.3f, %.3f, %.3f) turned %.0f deg: TCP at (%.3f, %.3f, %.3f), "
                "approach (%.2f, %.2f, %.2f)",
                spot.x(),
                spot.y(),
                spot.z(),
                turn * 180.0 / std::numbers::pi,
                release.translation().x(),
                release.translation().y(),
                release.translation().z(),
                approach.x(),
                approach.y(),
                approach.z());
            if (const auto at_release = motion_->ik(release); !at_release)
            {
                why = std::format("release pose: {}", at_release.error());
                RCLCPP_WARN(
                    get_logger(),
                    "Place turned %.0f deg: %s",
                    turn * 180.0 / std::numbers::pi,
                    why.c_str());
                continue;
            }
            const auto above_joints = motion_->ik(above);
            if (!above_joints)
            {
                why = std::format("above the release: {}", above_joints.error());
                RCLCPP_WARN(
                    get_logger(),
                    "Place turned %.0f deg: %s",
                    turn * 180.0 / std::numbers::pi,
                    why.c_str());
                continue;
            }
            phase("moving_above");
            if (const auto moved = motion_->toJoints(*above_joints); !moved)
            {
                why = moved.error();
                continue;
            }
            phase("descending");
            if (const auto down = motion_->linear(release); !down)
            {
                warnIfFailed(motion_->toJoints(*above_joints), "Backing up after a failed descent");
                why = down.error();
                continue;
            }
            phase("releasing");
            if (const auto opened = motion_->gripper(gripper_open_m_, gripper_effort_); !opened)
            {
                result.arm_stuck = true;
                return std::unexpected(opened.error());
            }
            detachHeld();
            phase("retreating");
            Eigen::Isometry3d clear = release;
            clear.translation().z() += std::max(place_approach_m_, clear_above_m_);
            if (const auto up = motion_->linear(clear); !up)
            {
                result.arm_stuck = true;
                return std::unexpected(up.error());
            }
            Eigen::Isometry3d placed = Eigen::Isometry3d::Identity();
            placed.translation() = spot;
            result.place_pose = stamped(placed, planning_frame_);
            return moveTo(ready_state_);
        }
        return std::unexpected(why);
    }

    Status runStage(
        const std::shared_ptr<Handle<piper_msgs::action::RunStage>> & goal,
        piper_msgs::action::RunStage::Result & result)
    {
        if (!stage_runner_)
        {
            return std::unexpected("no task: start the skills with task.file (task_file:=...)");
        }
        auto feedback = std::make_shared<piper_msgs::action::RunStage::Feedback>();
        const auto phase = [&](const char * name) {
            feedback->phase = name;
            goal->publish_feedback(feedback);
        };
        const auto ended = stage_runner_->run(
            *motion_, goal->get_goal()->stage, phase, [this] { return cancel_.load(); });
        if (!ended)
        {
            return std::unexpected(ended.error());
        }
        result.goal_error_m = *ended;
        return {};
    }

    void answerPlaceTarget(
        const piper_msgs::srv::PlaceTarget::Request & request,
        piper_msgs::srv::PlaceTarget::Response & response)
    {
        const std::optional<PlaceMode> mode = parsePlaceMode(request.mode);
        if (!mode)
        {
            response.success = false;
            response.message = std::format("place mode '{}' is unknown", request.mode);
            return;
        }
        std::optional<Eigen::Vector3d> anchor;
        if (!request.anchor_pose.header.frame_id.empty())
        {
            const auto & p = request.anchor_pose.pose.position;
            anchor = Eigen::Vector3d(p.x, p.y, p.z);
        }
        const auto target = placeTargetPoint(*mode, request.x, request.y, anchor, 0.0);
        if (!target)
        {
            response.success = false;
            response.message = target.error();
            return;
        }
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = *target;
        response.target = stamped(pose, planning_frame_);
        response.success = true;
        // Reachable: a top-down TCP a pre-grasp above the target has an IK solution. Checked only
        // while no skill runs: the skill's thread owns the motion helper meanwhile.
        const std::scoped_lock lock(motion_mutex_);
        if (motion_ && !busy_.load())
        {
            Eigen::Isometry3d probe = Eigen::Isometry3d::Identity();
            probe.linear() =
                Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX()).toRotationMatrix();
            probe.translation() = *target + Eigen::Vector3d(0.0, 0.0, pregrasp_m_);
            response.reachable = motion_->ik(probe).has_value();
        }
        response.message = response.reachable ? "reachable" : "not checked or out of reach";
    }

    // ------------------------------------------------------------------------------------------
    // State

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<CameraCapture> camera_;
    // Made at the first skill: MoveGroupInterface needs the node owned by a shared pointer. The
    // mutex guards the pointer itself (made by the worker, read by cancels and place_target).
    std::mutex motion_mutex_;
    std::unique_ptr<ArmMotion> motion_;
    rclcpp::CallbackGroup::SharedPtr actions_group_;
    rclcpp::CallbackGroup::SharedPtr clients_group_;
    rclcpp::Client<piper_msgs::srv::SegmentObjects>::SharedPtr segment_client_;
    rclcpp::Client<piper_msgs::srv::ComputeGrasps>::SharedPtr grasp_client_;
    rclcpp_action::Server<piper_msgs::action::MoveArmToNamedPose>::SharedPtr named_pose_server_;
    rclcpp_action::Server<piper_msgs::action::ViewWorkspace>::SharedPtr view_server_;
    rclcpp_action::Server<piper_msgs::action::FindObjects>::SharedPtr find_server_;
    rclcpp_action::Server<piper_msgs::action::PickObject>::SharedPtr pick_server_;
    rclcpp_action::Server<piper_msgs::action::PlaceObject>::SharedPtr place_server_;
    rclcpp_action::Server<piper_msgs::action::RunStage>::SharedPtr stage_server_;
    rclcpp::Service<piper_msgs::srv::PlaceTarget>::SharedPtr place_target_service_;
    std::atomic<bool> busy_{false};
    std::atomic<bool> cancel_{false};
    std::jthread worker_;
    // Written and read by the skill worker only.
    std::optional<Held> held_;

    std::string planning_frame_;
    std::string camera_frame_;
    std::map<std::string, JointValues> states_;
    std::string ready_state_;
    MotionSettings motion_settings_;
    CameraSettings camera_settings_;
    double gripper_open_m_{0.035};
    double gripper_closed_m_{0.0};
    double gripper_effort_{1.0};
    double held_min_m_{0.002};
    double width_tolerance_m_{0.02};
    std::vector<std::string> touch_links_;
    double held_padding_m_{0.005};
    TopRegion top_;
    TableAxes axes_;
    Eigen::Vector2d table_centre_{0.3, 0.0};
    Eigen::Vector2d table_size_{0.8, 1.2};
    double table_thickness_m_{0.05};
    // View name to the SRDF state that shows it.
    std::map<std::string, std::string> view_states_;
    ViewConfig view_;
    std::vector<std::string> preferred_views_;
    std::vector<TopView> views_;
    std::string segment_service_;
    std::string grasp_service_;
    std::chrono::milliseconds segment_timeout_{60000};
    std::chrono::milliseconds grasp_timeout_{60000};
    double default_min_score_{0.5};
    std::size_t min_points_{50};
    CloudOptions cloud_;
    double above_top_m_{0.006};
    double cluster_gap_m_{0.02};
    GraspFilter grasp_filter_;
    double pregrasp_m_{0.08};
    double lift_m_{0.08};
    std::size_t max_candidates_{15};
    int pick_attempts_{3};
    double release_height_m_{0.01};
    double place_approach_m_{0.08};
    double clear_above_m_{0.05};
    double place_tolerance_m_{0.05};
    FreeSpotOptions free_;
    // The task's stages, read with the parameters and handed to the runner.
    std::vector<Stage> task_stages_;
    StageSettings stage_settings_;
    std::unique_ptr<StageRunner> stage_runner_;
    // MoveIt's planning scene: the table, a held object and a task's parts.
    std::shared_ptr<SceneClient> scene_;
};

}  // namespace piper_skills

RCLCPP_COMPONENTS_REGISTER_NODE(piper_skills::SkillsNode)
