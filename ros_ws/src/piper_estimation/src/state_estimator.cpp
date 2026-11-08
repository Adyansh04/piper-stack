// Contact and grasp estimates from the arm's own feedback: the external wrench at the tool from
// the joint torques (WrenchEstimator) and whether the gripper holds something (GraspDetector).

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "piper_estimation/wrench_estimator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace piper_estimation
{

class StateEstimator : public rclcpp::Node
{
public:
    explicit StateEstimator(const rclcpp::NodeOptions & options)
        : Node("state_estimator", options)
        , base_link_(declare_parameter("base_link", std::string("base_link")))
        , tip_link_(declare_parameter("tip_link", std::string("tcp")))
        , joint_names_(declare_parameter(
              "joint_names",
              std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}))
        , gripper_joint_(declare_parameter("gripper_joint", std::string("joint7")))
        , grasp_(
              declare_parameter("grasp_effort_threshold_nm", 0.3),
              declare_parameter("grasp_min_opening_m", 0.004),
              static_cast<int>(declare_parameter("grasp_samples", 20)))
        , wrench_pub_(create_publisher<geometry_msgs::msg::WrenchStamped>("~/tcp_wrench", 10))
        , grasped_pub_(
              create_publisher<std_msgs::msg::Bool>(
                  "~/grasped", rclcpp::QoS(1).reliable().transient_local()))
        , description_sub_(
              create_subscription<std_msgs::msg::String>(
                  "/robot_description",
                  rclcpp::QoS(1).reliable().transient_local(),
                  [this](const std_msgs::msg::String & msg) { onDescription(msg.data); }))
        , joint_sub_(
              create_subscription<sensor_msgs::msg::JointState>(
                  "/joint_states",
                  rclcpp::SensorDataQoS(),
                  [this](const sensor_msgs::msg::JointState & msg) { onJointState(msg); }))
    {}

private:
    void onDescription(const std::string & urdf)
    {
        estimator_ = WrenchEstimator::fromUrdf(urdf, base_link_, tip_link_);
        if (!estimator_ || estimator_->joints() != joint_names_.size())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No %zu-joint chain from %s to %s in the robot description",
                joint_names_.size(),
                base_link_.c_str(),
                tip_link_.c_str());
            estimator_.reset();
        }
    }

    void onJointState(const sensor_msgs::msg::JointState & msg)
    {
        if (msg.effort.size() != msg.name.size() || msg.position.size() != msg.name.size())
        {
            return;
        }
        Eigen::VectorXd q(static_cast<Eigen::Index>(joint_names_.size()));
        Eigen::VectorXd tau(q.size());
        std::size_t found = 0;
        for (std::size_t i = 0; i < msg.name.size(); ++i)
        {
            if (msg.name[i] == gripper_joint_)
            {
                // joint7 is one finger's travel; the opening is twice it.
                const bool held = grasp_.update(msg.effort[i], 2.0 * msg.position[i]);
                if (!grasped_ || *grasped_ != held)
                {
                    grasped_ = held;
                    grasped_pub_->publish(std_msgs::msg::Bool().set__data(held));
                }
                continue;
            }
            for (std::size_t j = 0; j < joint_names_.size(); ++j)
            {
                if (msg.name[i] == joint_names_[j])
                {
                    q(static_cast<Eigen::Index>(j)) = msg.position[i];
                    tau(static_cast<Eigen::Index>(j)) = msg.effort[i];
                    ++found;
                }
            }
        }
        if (!estimator_ || found != joint_names_.size())
        {
            return;
        }
        const auto w = estimator_->wrench(q, tau);
        geometry_msgs::msg::WrenchStamped out;
        out.header.stamp = msg.header.stamp;
        out.header.frame_id = base_link_;
        out.wrench.force.x = w(0);
        out.wrench.force.y = w(1);
        out.wrench.force.z = w(2);
        out.wrench.torque.x = w(3);
        out.wrench.torque.y = w(4);
        out.wrench.torque.z = w(5);
        wrench_pub_->publish(out);
    }

    std::string base_link_;
    std::string tip_link_;
    std::vector<std::string> joint_names_;
    std::string gripper_joint_;
    GraspDetector grasp_;
    std::optional<WrenchEstimator> estimator_;
    std::optional<bool> grasped_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasped_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

}  // namespace piper_estimation

RCLCPP_COMPONENTS_REGISTER_NODE(piper_estimation::StateEstimator)
