// Runs a trained reach policy and moves the arm with it through MoveIt Servo, so Servo checks
// collisions, joint limits and singularities on every command.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <urdf/model.hpp>

#include "piper_controllers/joint_ramp.hpp"
#include "piper_controllers/policy.hpp"

namespace piper_controllers
{
namespace
{
using ServoCommandType = moveit_msgs::srv::ServoCommandType;
using SetBool = std_srvs::srv::SetBool;

/// Target position (m) then quaternion x, y, z, w, in the target frame.
using Target = std::array<float, 7>;

// Below this every joint counts as at rest after a stop, rad/s.
constexpr double kRestVelocityRadS = 1e-4;

/**
 * @brief Ramp steps per policy step: the policy steps a whole number of them, as in training.
 * @param[in] jog_period_s Length of a ramp step, s.
 * @param[in] policy_rate_hz Policy steps a second.
 * @throws std::runtime_error If the ramp rate is not a whole multiple of the policy rate.
 */
unsigned int decimationOf(double jog_period_s, double policy_rate_hz)
{
    const double steps = 1.0 / (jog_period_s * policy_rate_hz);
    const auto decimation = static_cast<unsigned int>(std::lround(steps));
    if (!std::isfinite(steps) || decimation < 1 || std::abs(steps - decimation) > 1e-6)
    {
        throw std::runtime_error("jog_rate_hz must be a whole multiple of policy_rate_hz");
    }
    return decimation;
}

/**
 * @brief Each joint's lower and upper position limit from a URDF, rad.
 * @param[in] description The URDF.
 * @param[in] joints The joints.
 * @throws std::runtime_error If no joint is named, the URDF does not parse or a joint has no
 * limits.
 */
std::pair<std::vector<double>, std::vector<double>> jointLimits(
    const std::string & description, const std::vector<std::string> & joints)
{
    if (joints.empty())
    {
        throw std::runtime_error("The parameter joints names no joint");
    }
    urdf::Model model;
    if (!model.initString(description))
    {
        throw std::runtime_error("The robot_description parameter does not parse as a URDF");
    }
    std::vector<double> lower_rad;
    std::vector<double> upper_rad;
    for (const auto & name : joints)
    {
        const auto joint = model.getJoint(name);
        if (!joint || !joint->limits)
        {
            throw std::runtime_error("Joint " + name + " has no limits in the description");
        }
        lower_rad.push_back(joint->limits->lower);
        upper_rad.push_back(joint->limits->upper);
    }
    return {std::move(lower_rad), std::move(upper_rad)};
}

/**
 * @brief Loads a policy that reads a reach observation of a number of joints.
 * @param[in] model_path The ONNX file.
 * @param[in] joints Number of joints.
 * @throws std::runtime_error If the policy does not load or its sizes differ.
 */
Policy loadPolicy(const std::string & model_path, std::size_t joints)
{
    auto policy = Policy::load(model_path, 2 * joints + std::tuple_size_v<Target>, joints);
    if (!policy)
    {
        throw std::runtime_error(policy.error());
    }
    return std::move(*policy);
}

/**
 * @class PolicyRunner
 * @brief Moves the arm by a trained reach policy through MoveIt Servo's joint jogs.
 * @details While active, every `1 / jog_rate_hz` a JointRamp moves each joint's command towards
 * the position the policy wants (`action_offset` plus `action_scale` times its action) under
 * `max_velocity` and `max_acceleration`, from the measured positions at activation; the ramp's
 * velocities go to Servo as a JointJog. Every `jog_rate_hz / policy_rate_hz` steps the policy reads
 * [joint positions (rad), joint velocities (rad/s), target position (m), target quaternion x y z
 * w, in `frame`] and gives the actions. This is the motion the reach task in `lab/` trains with.
 *
 * The joints hold until a target arrives on `~/target_pose` after activation; a target in
 * another frame is ignored, and the joints come to rest if inference fails. `~/set_active` true
 * switches Servo to joint jogs and starts; false brings the joints to rest under the caps, then
 * stops sending.
 *
 * @warning The policy was trained with the offsets, scale, caps and rates in its YAML: other
 * values change what it does.
 */
class PolicyRunner : public rclcpp::Node
{
public:
    /**
     * @param[in] options The node's options.
     * @throws std::runtime_error If a parameter is inconsistent, the robot description lacks a
     * joint's limits, or the policy does not load.
     */
    explicit PolicyRunner(const rclcpp::NodeOptions & options)
        : Node("policy_runner", options)
        , joints_(declare_parameter("joints", std::vector<std::string>{}))
        , frame_(declare_parameter("frame", std::string("base_link")))
        , model_path_(declare_parameter("model_path", std::string()))
        , jog_period_s_(1.0 / declare_parameter("jog_rate_hz", 200.0))
        , decimation_(decimationOf(jog_period_s_, declare_parameter("policy_rate_hz", 50.0)))
        , ramp_(makeRamp())
        , policy_(loadPolicy(model_path_, joints_.size()))
    {
        const std::size_t joints = joints_.size();
        for (std::size_t i = 0; i < joints; ++i)
        {
            index_.emplace(joints_[i], i);
        }
        positions_rad_.assign(joints, 0.0);
        velocities_rad_s_.assign(joints, 0.0);
        seen_.assign(joints, false);
        jog_.joint_names = joints_;
        jog_pub_ = create_publisher<control_msgs::msg::JointJog>(
            declare_parameter("jog_topic", std::string("/servo_node/delta_joint_cmds")), 10);
        command_type_ = create_client<ServoCommandType>(declare_parameter(
            "command_type_service", std::string("/servo_node/switch_command_type")));
        state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState & msg) { onState(msg); });
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/target_pose",
            rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::PoseStamped & pose) { onTarget(pose); });
        active_srv_ = create_service<SetBool>(
            "~/set_active",
            [this](
                const std::shared_ptr<SetBool::Request> & request,
                const std::shared_ptr<SetBool::Response> & response) {
                onSetActive(*request, *response);
            });
        // On the ROS clock: in simulation the jogs keep the simulator's time, which Servo needs.
        timer_ = rclcpp::create_timer(
            this, get_clock(), rclcpp::Duration::from_seconds(jog_period_s_), [this] { step(); });
        RCLCPP_INFO(
            get_logger(),
            "Policy %s on %zu joints, one step every %u jogs: inactive",
            model_path_.c_str(),
            joints,
            decimation_);
    }

private:
    /// @brief The ramp of the joints, with their limits from the robot description.
    JointRamp makeRamp()
    {
        auto [lower_rad, upper_rad] =
            jointLimits(declare_parameter("robot_description", std::string()), joints_);
        return {
            std::move(lower_rad),
            std::move(upper_rad),
            declare_parameter("action_offset", std::vector<double>{}),
            declare_parameter("action_scale", 0.5),
            declare_parameter("max_velocity", 1.0),
            declare_parameter("max_acceleration", 0.5)};
    }

    void onState(const sensor_msgs::msg::JointState & msg)
    {
        for (std::size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k)
        {
            const auto it = index_.find(msg.name[k]);
            if (it == index_.end())
            {
                continue;
            }
            positions_rad_[it->second] = msg.position[k];
            velocities_rad_s_[it->second] = k < msg.velocity.size() ? msg.velocity[k] : 0.0;
            seen_[it->second] = true;
        }
    }

    void onTarget(const geometry_msgs::msg::PoseStamped & pose)
    {
        if (pose.header.frame_id != frame_)
        {
            RCLCPP_WARN(
                get_logger(),
                "Ignored a target in %s: targets are in %s",
                pose.header.frame_id.c_str(),
                frame_.c_str());
            return;
        }
        const auto & p = pose.pose.position;
        const auto & q = pose.pose.orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(p.x + p.y + p.z) || !(norm > 1e-6) || !std::isfinite(norm))
        {
            RCLCPP_WARN(
                get_logger(), "Ignored a target with a non-finite position or no orientation");
            return;
        }
        if (!active_)
        {
            RCLCPP_WARN(get_logger(), "Ignored a target while inactive: call ~/set_active first");
            return;
        }
        target_ = Target{
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z),
            static_cast<float>(q.x / norm),
            static_cast<float>(q.y / norm),
            static_cast<float>(q.z / norm),
            static_cast<float>(q.w / norm)};
    }

    void onSetActive(const SetBool::Request & request, SetBool::Response & response)
    {
        if (!request.data)
        {
            // The joints come to rest under the caps; step() stops sending once they are.
            ramp_.stop();
            target_.reset();
            stopping_ = active_;
            active_ = false;
            response.success = true;
            response.message = "Stopping";
            return;
        }
        if (!std::ranges::all_of(seen_, [](bool seen) { return seen; }))
        {
            response.success = false;
            response.message = "No joint state yet for every joint";
            return;
        }
        if (!command_type_->service_is_ready())
        {
            response.success = false;
            response.message = "MoveIt Servo is not running";
            return;
        }
        auto switch_request = std::make_shared<ServoCommandType::Request>();
        switch_request->command_type = ServoCommandType::Request::JOINT_JOG;
        command_type_->async_send_request(switch_request);
        ramp_.reset(positions_rad_);
        target_.reset();
        cycle_ = 0;
        active_ = true;
        stopping_ = false;
        response.success = true;
        response.message = "Active: send a target on ~/target_pose";
    }

    void step()
    {
        if (!active_ && !stopping_)
        {
            return;
        }
        Policy & policy = policy_;
        JointRamp & ramp = ramp_;
        if (active_ && !target_)
        {
            ramp.stop();
        }
        else if (active_ && cycle_ == 0)
        {
            const Target & target = *target_;
            const std::span<float> observations = policy.observations();
            const std::size_t joints = joints_.size();
            for (std::size_t i = 0; i < joints; ++i)
            {
                observations[i] = static_cast<float>(positions_rad_[i]);
                observations[joints + i] = static_cast<float>(velocities_rad_s_[i]);
            }
            std::ranges::copy(
                target, std::next(observations.begin(), static_cast<std::ptrdiff_t>(2 * joints)));
            if (policy.run())
            {
                ramp.want(policy.actions());
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000, "Inference failed: the joints come to rest");
                ramp.stop();
            }
        }
        cycle_ = (cycle_ + 1) % decimation_;
        std::ignore = ramp.step(jog_period_s_);
        const std::span<const double> velocities = ramp.velocities();
        const rclcpp::Time now = get_clock()->now();
        // Servo aborts on a jog stamped zero: the simulator's clock has not come yet.
        if (now.nanoseconds() == 0)
        {
            return;
        }
        jog_.header.stamp = now;
        jog_.velocities.assign(velocities.begin(), velocities.end());
        jog_pub_->publish(jog_);
        if (stopping_ && std::ranges::all_of(velocities, [](double v) {
                return std::abs(v) < kRestVelocityRadS;
            }))
        {
            stopping_ = false;
            RCLCPP_INFO(get_logger(), "At rest: inactive");
        }
    }

    std::vector<std::string> joints_;
    std::string frame_;
    std::string model_path_;
    double jog_period_s_;
    // Ramp steps per policy step, and where in that count this step is.
    unsigned int decimation_;
    unsigned int cycle_ = 0;
    JointRamp ramp_;
    Policy policy_;
    std::unordered_map<std::string, std::size_t> index_;
    std::vector<double> positions_rad_;
    std::vector<double> velocities_rad_s_;
    std::vector<bool> seen_;
    std::optional<Target> target_;
    bool active_ = false;
    // Inactive but still bringing the joints to rest.
    bool stopping_ = false;
    control_msgs::msg::JointJog jog_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
    rclcpp::Client<ServoCommandType>::SharedPtr command_type_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Service<SetBool>::SharedPtr active_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace piper_controllers

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int status = 0;
    try
    {
        rclcpp::spin(std::make_shared<piper_controllers::PolicyRunner>(rclcpp::NodeOptions()));
    }
    catch (const std::exception & error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("policy_runner"), "%s", error.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
