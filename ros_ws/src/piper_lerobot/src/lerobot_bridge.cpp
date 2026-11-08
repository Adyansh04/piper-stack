// Serves LeRobot's plugins, which run in their own Python 3.12 environment outside ROS, over ZMQ:
// the arm's joint state and wrist image, and joint targets that become MoveIt Servo joint jogs
// (collision and limit checked) and gripper goals.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <zmq.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "piper_lerobot/bridge_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace piper_lerobot
{

using GripperCommand = control_msgs::action::ParallelGripperCommand;
using ServoCommandType = moveit_msgs::srv::ServoCommandType;

namespace
{

std::vector<std::string> withGripper(std::vector<std::string> arm, const std::string & gripper)
{
    arm.push_back(gripper);
    return arm;
}

}  // namespace

class LeRobotBridge : public rclcpp::Node
{
public:
    explicit LeRobotBridge(const rclcpp::NodeOptions & options)
        : Node("lerobot_bridge", options)
        , arm_joints_(declare_parameter(
              "arm_joints",
              std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}))
        , gripper_joint_(declare_parameter("gripper_joint", std::string("joint7")))
        , gain_(declare_parameter("jog_gain", 2.0))
        , max_velocity_rad_s_(declare_parameter("max_jog_velocity", 0.5))
        , target_timeout_s_(declare_parameter("target_timeout_s", 0.5))
        , gripper_deadband_m_(declare_parameter("gripper_deadband_m", 0.002))
        , gripper_effort_nm_(declare_parameter("gripper_effort_nm", 1.0))
        , endpoint_(declare_parameter("endpoint", std::string("tcp://127.0.0.1:5590")))
        , joints_(withGripper(arm_joints_, gripper_joint_))
        , measured_(joints_.size(), std::numeric_limits<double>::quiet_NaN())
        , commanded_(joints_.size(), std::numeric_limits<double>::quiet_NaN())
        , jog_pub_(
              create_publisher<control_msgs::msg::JointJog>(
                  declare_parameter("jog_topic", std::string("/servo_node/delta_joint_cmds")),
                  rclcpp::SystemDefaultsQoS()))
        , command_type_(
              create_client<ServoCommandType>(declare_parameter(
                  "command_type_service", std::string("/servo_node/switch_command_type"))))
        , gripper_(
              rclcpp_action::create_client<GripperCommand>(
                  this,
                  declare_parameter(
                      "gripper_action", std::string("/gripper_controller/gripper_cmd"))))
    {
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            declare_parameter("joint_state_topic", std::string("/joint_states")),
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState & msg) { onJointState(msg); });
        reference_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            declare_parameter(
                "controller_state_topic", std::string("/arm_controller/controller_state")),
            rclcpp::SensorDataQoS(),
            [this](const control_msgs::msg::JointTrajectoryControllerState & msg) {
                onControllerState(msg);
            });
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            declare_parameter("image_topic", std::string("/wrist_camera/color/image_raw")),
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(std::move(msg)); });
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / declare_parameter("jog_rate_hz", 100.0)),
            [this] { jog(); });
        server_ = std::jthread([this](const std::stop_token & stop) { serve(stop); });
        RCLCPP_INFO(get_logger(), "LeRobot bridge on %s", endpoint_.c_str());
    }

private:
    void onJointState(const sensor_msgs::msg::JointState & msg)
    {
        if (msg.position.size() != msg.name.size())
        {
            return;
        }
        const std::scoped_lock lock(mutex_);
        for (std::size_t i = 0; i < msg.name.size(); ++i)
        {
            const auto joint = std::ranges::find(joints_, msg.name[i]);
            if (joint != joints_.end())
            {
                measured_[static_cast<std::size_t>(std::distance(joints_.begin(), joint))] =
                    msg.position[i];
            }
        }
        // The gripper controller publishes no command: its measured opening stands in.
        commanded_.back() = measured_.back();
        stamp_s_ = rclcpp::Time(msg.header.stamp).seconds();
    }

    void onControllerState(const control_msgs::msg::JointTrajectoryControllerState & msg)
    {
        if (msg.reference.positions.size() != msg.joint_names.size())
        {
            return;
        }
        const std::scoped_lock lock(mutex_);
        for (std::size_t i = 0; i < msg.joint_names.size(); ++i)
        {
            const auto joint = std::ranges::find(arm_joints_, msg.joint_names[i]);
            if (joint != arm_joints_.end())
            {
                commanded_[static_cast<std::size_t>(std::distance(arm_joints_.begin(), joint))] =
                    msg.reference.positions[i];
            }
        }
    }

    void onImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        if (msg->encoding != "rgb8" && msg->encoding != "bgr8")
        {
            RCLCPP_WARN_ONCE(
                get_logger(), "Images in %s are ignored: rgb8 or bgr8 only", msg->encoding.c_str());
            return;
        }
        const std::scoped_lock lock(mutex_);
        image_ = std::move(msg);
    }

    void jog()
    {
        std::vector<double> target;
        std::vector<double> measured;
        bool switch_to_joint_jog = false;
        {
            const std::scoped_lock lock(mutex_);
            if (!active_)
            {
                return;
            }
            if ((now() - target_stamp_).seconds() > target_timeout_s_)
            {
                // No action for a while: the client stopped; the arm stops with Servo's ramp.
                active_ = false;
                target.clear();
            }
            else
            {
                target = target_;
                measured = measured_;
                switch_to_joint_jog = std::exchange(switch_command_type_, false);
            }
        }
        if (switch_to_joint_jog)
        {
            auto request = std::make_shared<ServoCommandType::Request>();
            request->command_type = ServoCommandType::Request::JOINT_JOG;
            command_type_->async_send_request(request);
        }
        const rclcpp::Time stamp = now();
        if (stamp.nanoseconds() == 0)
        {
            // The simulator's clock has not come yet, and Servo aborts on a jog stamped zero.
            return;
        }
        control_msgs::msg::JointJog msg;
        msg.header.stamp = stamp;
        msg.joint_names = arm_joints_;
        msg.velocities.assign(arm_joints_.size(), 0.0);
        if (!target.empty()
            && std::ranges::none_of(measured, [](double value) { return std::isnan(value); }))
        {
            const std::size_t arm = arm_joints_.size();
            msg.velocities = jogVelocities(
                std::span(target).first(arm),
                std::span(measured).first(arm),
                gain_,
                max_velocity_rad_s_);
            moveGripper(target.back());
        }
        jog_pub_->publish(msg);
    }

    void moveGripper(double position_m)
    {
        if (std::abs(position_m - gripper_goal_m_) < gripper_deadband_m_
            || !gripper_->action_server_is_ready())
        {
            return;
        }
        gripper_goal_m_ = position_m;
        GripperCommand::Goal goal;
        goal.command.name = {gripper_joint_};
        goal.command.position = {position_m};
        goal.command.effort = {gripper_effort_nm_};
        gripper_->async_send_goal(goal);
    }

    // The ZMQ socket lives and dies in this thread: a socket belongs to one thread.
    void serve(const std::stop_token & stop)
    {
        zmq::socket_t socket(context_, zmq::socket_type::rep);
        socket.set(zmq::sockopt::linger, 0);
        socket.bind(endpoint_);
        while (!stop.stop_requested())
        {
            zmq::pollitem_t item{
                .socket = socket.handle(), .fd = 0, .events = ZMQ_POLLIN, .revents = 0};
            try
            {
                if (zmq::poll(&item, 1, std::chrono::milliseconds(100)) == 0)
                {
                    continue;
                }
                std::vector<zmq::message_t> request;
                static_cast<void>(zmq::recv_multipart(socket, std::back_inserter(request)));
                zmq::send_multipart(socket, handle(request));
            }
            catch (const zmq::error_t & error)
            {
                RCLCPP_ERROR(get_logger(), "ZMQ error on %s: %s", endpoint_.c_str(), error.what());
            }
        }
    }

    // Every request gets a reply, a refusal included: a lost reply strands the client's socket.
    std::vector<zmq::message_t> handle(const std::vector<zmq::message_t> & request)
    {
        const std::string_view command = request.empty() ? "" : request[0].to_string_view();
        if (command == protocol::kObserve)
        {
            return observe();
        }
        if (command == protocol::kAct && request.size() == 2)
        {
            const auto positions = decodePositions(
                std::span(static_cast<const std::byte *>(request[1].data()), request[1].size()),
                joints_.size());
            if (!positions)
            {
                return refuse(positions.error());
            }
            const std::scoped_lock lock(mutex_);
            target_ = *positions;
            target_stamp_ = now();
            if (!active_)
            {
                active_ = true;
                switch_command_type_ = true;
            }
            return ok();
        }
        if (command == protocol::kRelease)
        {
            const std::scoped_lock lock(mutex_);
            target_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            return ok();
        }
        return refuse(std::format("unknown request '{}'", command));
    }

    std::vector<zmq::message_t> observe()
    {
        sensor_msgs::msg::Image::ConstSharedPtr image;
        std::vector<std::byte> state;
        {
            const std::scoped_lock lock(mutex_);
            if (std::ranges::any_of(measured_, [](double value) { return std::isnan(value); }))
            {
                return refuse("no joint state yet");
            }
            if (!image_)
            {
                return refuse("no camera image yet");
            }
            image = image_;
            state = encodeState(stamp_s_, measured_, commanded_);
        }
        const auto size = encodeImageSize(image->height, image->width);
        zmq::message_t pixels(static_cast<std::size_t>(image->height) * image->width * 3);
        auto * out = static_cast<std::uint8_t *>(pixels.data());
        const bool bgr = image->encoding == "bgr8";
        for (std::uint32_t row = 0; row < image->height; ++row)
        {
            const std::uint8_t * in = &image->data[static_cast<std::size_t>(row) * image->step];
            for (std::uint32_t col = 0; col < image->width; ++col, in += 3, out += 3)
            {
                out[0] = bgr ? in[2] : in[0];
                out[1] = in[1];
                out[2] = bgr ? in[0] : in[2];
            }
        }
        std::vector<zmq::message_t> reply;
        reply.emplace_back(protocol::kOk);
        reply.emplace_back(state.data(), state.size());
        reply.emplace_back(size.data(), size.size());
        reply.push_back(std::move(pixels));
        return reply;
    }

    static std::vector<zmq::message_t> ok()
    {
        std::vector<zmq::message_t> reply;
        reply.emplace_back(protocol::kOk);
        return reply;
    }

    static std::vector<zmq::message_t> refuse(std::string_view reason)
    {
        std::vector<zmq::message_t> reply;
        reply.emplace_back(protocol::kError);
        reply.emplace_back(reason);
        return reply;
    }

    std::vector<std::string> arm_joints_;
    std::string gripper_joint_;
    double gain_;
    double max_velocity_rad_s_;
    double target_timeout_s_;
    double gripper_deadband_m_;
    double gripper_effort_nm_;
    std::string endpoint_;
    // The arm's joints then the gripper's: the order of every position frame.
    std::vector<std::string> joints_;
    double gripper_goal_m_ = std::numeric_limits<double>::quiet_NaN();

    // Guards everything below, shared by the executor's callbacks and the ZMQ thread.
    std::mutex mutex_;
    std::vector<double> measured_;
    std::vector<double> commanded_;
    double stamp_s_ = 0.0;
    sensor_msgs::msg::Image::ConstSharedPtr image_;
    std::vector<double> target_;
    rclcpp::Time target_stamp_;
    bool active_ = false;
    bool switch_command_type_ = false;

    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
    rclcpp::Client<ServoCommandType>::SharedPtr command_type_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
        reference_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    zmq::context_t context_{1};
    // Last: destroyed first, so the server thread stops before what it uses goes.
    std::jthread server_;
};

}  // namespace piper_lerobot

RCLCPP_COMPONENTS_REGISTER_NODE(piper_lerobot::LeRobotBridge)
