// Keeps MoveIt's planning scene in step with a task's parts, so planning and MoveIt Servo know
// where the fixture and every part are, which part the gripper holds and which contacts a stage
// allows.

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <piper_msgs/srv/allow_contacts.hpp>
#include <piper_msgs/srv/attach_part.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "piper_skills/scene_client.hpp"

namespace piper_skills
{
namespace
{
using AllowContacts = piper_msgs::srv::AllowContacts;
using AttachPart = piper_msgs::srv::AttachPart;

// The task's CAD meshes are in millimetres.
constexpr double kMeshScale = 1e-3;
// move_group answers a scene request in milliseconds; longer means it is gone.
constexpr std::chrono::seconds kSceneTimeout{2};

/**
 * @brief Each part's collision meshes, by part name, from a task's task.yaml.
 * @param[in] task_file The task.yaml; mesh paths are relative to its folder.
 * @throws std::runtime_error If the file does not parse, has no parts or a mesh does not load.
 */
std::unordered_map<std::string, std::vector<shape_msgs::msg::Mesh>> loadParts(
    const std::string & task_file)
{
    YAML::Node spec;
    try
    {
        spec = YAML::LoadFile(task_file);
    }
    catch (const YAML::Exception & error)
    {
        throw std::runtime_error("Task file " + task_file + ": " + error.what());
    }
    if (!spec["parts"] || !spec["parts"].IsMap())
    {
        throw std::runtime_error("Task file " + task_file + " has no parts");
    }
    const std::filesystem::path root = std::filesystem::path(task_file).parent_path();
    std::unordered_map<std::string, std::vector<shape_msgs::msg::Mesh>> parts;
    for (const auto & part : spec["parts"])
    {
        auto & meshes = parts[part.first.as<std::string>()];
        for (const auto & shape : part.second["shapes"])
        {
            // A shape without collision (a look only) is no obstacle.
            if (shape["collision"].as<std::string>() == "none")
            {
                continue;
            }
            const std::string path = (root / shape["mesh"].as<std::string>()).string();
            const std::unique_ptr<shapes::Mesh> mesh(
                shapes::createMeshFromResource(
                    "file://" + path, Eigen::Vector3d::Constant(kMeshScale)));
            if (!mesh)
            {
                throw std::runtime_error("Mesh " + path + " does not load");
            }
            shapes::ShapeMsg message;
            shapes::constructMsgFromShape(mesh.get(), message);
            // NOLINTNEXTLINE(misc-include-cleaner): ShapeMsg is boost::variant, its get comes with it
            meshes.push_back(boost::get<shape_msgs::msg::Mesh>(message));
        }
    }
    // A part only seen (no collision shape) is no obstacle: MoveIt refuses an empty object.
    std::erase_if(parts, [](const auto & part) { return part.second.empty(); });
    return parts;
}

/// @brief Whether two poses differ by more than a distance or an angle.
bool moved(
    const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double distance_m, double angle_rad)
{
    return (a.translation() - b.translation()).norm() > distance_m
           || Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle() > angle_rad;
}

/**
 * @class TaskScene
 * @brief Keeps MoveIt's planning scene in step with a task's parts.
 * @details Every part of `task_file` with collision shapes becomes a collision object of its
 * meshes, at the pose last given for it on `poses_topic` (`tf2_msgs/TFMessage`, child frame = the
 * part's name, in `planning_frame`). Every `update_period_s` a part that moved more than
 * `position_tolerance_m` or `angle_tolerance_rad` is moved in the scene. `~/attach` attaches a part
 * to `tcp_link` (it may touch `touch_links`) and stops its updates until it is detached.
 * `~/allow_contacts` allows or forbids contacts between pairs of scene objects or robot links in
 * the allowed collision matrix that MoveIt's planning and Servo check against. Changes go to
 * move_group's `/apply_planning_scene`; a part is shown once move_group has taken it.
 *
 * @note Runs in a multi-threaded executor (`component_container_mt`): its callbacks wait for
 * move_group's answers, which come on a callback group of their own.
 */
class TaskScene : public rclcpp::Node
{
public:
    /**
     * @param[in] options The node's options.
     * @throws std::runtime_error If the task's parts or meshes do not load.
     */
    explicit TaskScene(const rclcpp::NodeOptions & options)
        : Node("task_scene", options)
        , parts_(loadParts(declare_parameter("task_file", std::string())))
        , frame_(declare_parameter("planning_frame", std::string("world")))
        , tcp_link_(declare_parameter("tcp_link", std::string("tcp")))
        , touch_links_(declare_parameter("touch_links", std::vector<std::string>{}))
        , distance_m_(declare_parameter("position_tolerance_m", 0.0001))
        , angle_rad_(declare_parameter("angle_tolerance_rad", 0.002))
    {
        poses_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
            declare_parameter("poses_topic", std::string("/sim/objects")),
            rclcpp::QoS(10).reliable(),
            [this](const tf2_msgs::msg::TFMessage & msg) { onPoses(msg); });
        // The callbacks wait for move_group: its answers come on a group of their own.
        clients_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        scene_ = std::make_unique<SceneClient>(*this, clients_, kSceneTimeout);
        attach_srv_ = create_service<AttachPart>(
            "~/attach",
            [this](
                const std::shared_ptr<AttachPart::Request> & request,
                const std::shared_ptr<AttachPart::Response> & response) {
                onAttach(*request, *response);
            });
        allow_srv_ = create_service<AllowContacts>(
            "~/allow_contacts",
            [this](
                const std::shared_ptr<AllowContacts::Request> & request,
                const std::shared_ptr<AllowContacts::Response> & response) {
                onAllow(*request, *response);
            });
        timer_ = create_wall_timer(
            std::chrono::duration<double>(declare_parameter("update_period_s", 0.1)),
            [this] { update(); });
        RCLCPP_INFO(get_logger(), "Task scene: %zu parts", parts_.size());
    }

private:
    void onPoses(const tf2_msgs::msg::TFMessage & msg)
    {
        for (const auto & tf : msg.transforms)
        {
            if (!parts_.contains(tf.child_frame_id))
            {
                continue;
            }
            if (tf.header.frame_id != frame_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Ignored poses in %s: the scene is in %s",
                    tf.header.frame_id.c_str(),
                    frame_.c_str());
                continue;
            }
            latest_[tf.child_frame_id] = tf2::transformToEigen(tf.transform);
        }
    }

    void update()
    {
        moveit_msgs::msg::PlanningScene diff;
        std::vector<std::string> names;
        for (const auto & [name, pose] : latest_)
        {
            const auto shown = shown_.find(name);
            if (attached_.contains(name)
                || (shown != shown_.end() && !moved(shown->second, pose, distance_m_, angle_rad_)))
            {
                continue;
            }
            moveit_msgs::msg::CollisionObject object;
            object.id = name;
            object.header.frame_id = frame_;
            object.pose = tf2::toMsg(pose);
            if (shown == shown_.end())
            {
                object.meshes = parts_.at(name);
                // Identity poses: the meshes are in the part's frame.
                object.mesh_poses.resize(object.meshes.size());
                object.operation = moveit_msgs::msg::CollisionObject::ADD;
            }
            else
            {
                object.operation = moveit_msgs::msg::CollisionObject::MOVE;
            }
            diff.world.collision_objects.push_back(std::move(object));
            names.push_back(name);
        }
        if (names.empty())
        {
            return;
        }
        if (const auto applied = scene_->apply(std::move(diff)); !applied)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "The parts' poses: %s", applied.error().c_str());
            return;
        }
        for (const auto & name : names)
        {
            shown_[name] = latest_.at(name);
        }
    }

    void onAttach(const AttachPart::Request & request, AttachPart::Response & response)
    {
        if (!shown_.contains(request.part))
        {
            response.success = false;
            response.message = "No part " + request.part + " in the scene";
            return;
        }
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = tcp_link_;
        attached.touch_links = touch_links_;
        attached.object.id = request.part;
        attached.object.header.frame_id = frame_;
        attached.object.operation = request.attach ? moveit_msgs::msg::CollisionObject::ADD
                                                   : moveit_msgs::msg::CollisionObject::REMOVE;
        moveit_msgs::msg::PlanningScene diff;
        diff.robot_state.is_diff = true;
        diff.robot_state.attached_collision_objects.push_back(std::move(attached));
        if (const auto applied = scene_->apply(std::move(diff)); !applied)
        {
            response.success = false;
            response.message = applied.error();
            return;
        }
        if (request.attach)
        {
            attached_.insert(request.part);
        }
        else
        {
            // Back in the world where it is: the next pose moves it from there.
            attached_.erase(request.part);
            shown_.erase(request.part);
        }
        response.success = true;
        response.message = request.attach ? "Attached" : "Detached";
    }

    void onAllow(const AllowContacts::Request & request, AllowContacts::Response & response)
    {
        if (request.first.size() != request.second.size() || request.first.empty())
        {
            response.success = false;
            response.message = "first and second must name the same, non-zero number of objects";
            return;
        }
        // The matrix goes back whole: a diff with a matrix replaces the scene's.
        const auto current =
            scene_->get(moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX);
        if (!current)
        {
            response.success = false;
            response.message = current.error();
            return;
        }
        collision_detection::AllowedCollisionMatrix acm(current->allowed_collision_matrix);
        for (std::size_t i = 0; i < request.first.size(); ++i)
        {
            acm.setEntry(request.first[i], request.second[i], request.allowed);
        }
        moveit_msgs::msg::PlanningScene diff;
        acm.getMessage(diff.allowed_collision_matrix);
        if (const auto applied = scene_->apply(std::move(diff)); !applied)
        {
            response.success = false;
            response.message = applied.error();
            return;
        }
        response.success = true;
        response.message = request.allowed ? "Allowed" : "Forbidden";
    }

    std::unordered_map<std::string, std::vector<shape_msgs::msg::Mesh>> parts_;
    std::string frame_;
    std::string tcp_link_;
    std::vector<std::string> touch_links_;
    double distance_m_;
    double angle_rad_;
    // The newest pose of each part, and the pose the scene shows it at.
    std::unordered_map<std::string, Eigen::Isometry3d> latest_;
    std::unordered_map<std::string, Eigen::Isometry3d> shown_;
    std::set<std::string> attached_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr poses_sub_;
    rclcpp::CallbackGroup::SharedPtr clients_;
    std::unique_ptr<SceneClient> scene_;
    rclcpp::Service<AttachPart>::SharedPtr attach_srv_;
    rclcpp::Service<AllowContacts>::SharedPtr allow_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace piper_skills

RCLCPP_COMPONENTS_REGISTER_NODE(piper_skills::TaskScene)
