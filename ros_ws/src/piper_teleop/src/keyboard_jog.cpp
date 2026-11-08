// Keyboard jogging of the arm through MoveIt Servo: keys become twists and joint jogs on Servo's
// input topics, the same ones a gamepad or SpaceMouse node would publish instead. Needs a
// terminal, so it runs with `ros2 run` (see the package README), not from a launch file.

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <memory>
#include <print>
#include <string>
#include <vector>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "piper_teleop/key_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using piper_teleop::KeyAction;
using ServoCommandType = moveit_msgs::srv::ServoCommandType;
using GripperCommand = control_msgs::action::ParallelGripperCommand;

// Non-canonical, unechoed input for as long as it lives; restores the terminal on every exit.
class RawTerminal
{
public:
    RawTerminal()
    {
        if (active_)
        {
            termios raw = saved_;
            raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }
    RawTerminal(const RawTerminal &) = delete;
    RawTerminal & operator=(const RawTerminal &) = delete;
    RawTerminal(RawTerminal &&) = delete;
    RawTerminal & operator=(RawTerminal &&) = delete;
    ~RawTerminal()
    {
        if (active_)
        {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        }
    }
    [[nodiscard]] bool active() const noexcept
    {
        return active_;
    }

private:
    termios saved_{};
    bool active_ = ::isatty(STDIN_FILENO) == 1 && ::tcgetattr(STDIN_FILENO, &saved_) == 0;
};

class KeyboardJog : public rclcpp::Node
{
public:
    KeyboardJog()
        : Node("keyboard_jog")
        , linear_speed_mps_(declare_parameter("linear_speed_mps", 0.05))
        , angular_speed_radps_(declare_parameter("angular_speed_radps", 0.3))
        , joint_speed_radps_(declare_parameter("joint_speed_radps", 0.3))
        , key_hold_s_(declare_parameter("key_hold_s", 0.15))
        , base_frame_(declare_parameter("base_frame", std::string("base_link")))
        , tool_frame_(declare_parameter("tool_frame", std::string("tcp")))
        , joint_names_(declare_parameter(
              "joint_names",
              std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}))
        , gripper_joint_(declare_parameter("gripper_joint", std::string("joint7")))
        , gripper_open_m_(declare_parameter("gripper_open_m", 0.035))
        , gripper_closed_m_(declare_parameter("gripper_closed_m", 0.0))
        , gripper_effort_nm_(declare_parameter("gripper_effort_nm", 1.0))
        , twist_pub_(
              create_publisher<geometry_msgs::msg::TwistStamped>(
                  declare_parameter("twist_topic", std::string("/servo_node/delta_twist_cmds")),
                  rclcpp::SystemDefaultsQoS()))
        , jog_pub_(
              create_publisher<control_msgs::msg::JointJog>(
                  declare_parameter("joint_jog_topic", std::string("/servo_node/delta_joint_cmds")),
                  rclcpp::SystemDefaultsQoS()))
        , command_type_(
              create_client<ServoCommandType>(declare_parameter(
                  "command_type_service", std::string("/servo_node/switch_command_type"))))
        , gripper_(
              rclcpp_action::create_client<GripperCommand>(
                  this,
                  declare_parameter(
                      "gripper_action", std::string("/gripper_controller/gripper_cmd"))))
        , timer_(create_wall_timer(
              std::chrono::duration<double>(1.0 / declare_parameter("publish_rate_hz", 50.0)),
              [this] { tick(); }))
    {}

    [[nodiscard]] bool done() const noexcept
    {
        return done_;
    }

private:
    void tick()
    {
        char key = 0;
        while (::read(STDIN_FILENO, &key, 1) == 1)
        {
            handle(piper_teleop::keyAction(key));
        }
        if (now() < active_until_)
        {
            publish();
        }
    }

    void handle(const KeyAction & action)
    {
        using Kind = KeyAction::Kind;
        switch (action.kind)
        {
            case Kind::kTwist:
                twist_ = action.twist;
                hold(ServoCommandType::Request::TWIST);
                break;
            case Kind::kSelectJoint:
                joint_ = static_cast<std::size_t>(action.joint - 1);
                std::println("jogging {}", joint_names_.at(joint_));
                break;
            case Kind::kJointJog:
                joint_direction_ = action.direction;
                hold(ServoCommandType::Request::JOINT_JOG);
                break;
            case Kind::kGripperOpen:
                moveGripper(gripper_open_m_);
                break;
            case Kind::kGripperClose:
                moveGripper(gripper_closed_m_);
                break;
            case Kind::kToggleFrame:
                tool_frame_active_ = !tool_frame_active_;
                std::println("twists in {}", tool_frame_active_ ? tool_frame_ : base_frame_);
                break;
            case Kind::kFaster:
            case Kind::kSlower:
                scale_ =
                    std::clamp(scale_ * (action.kind == Kind::kFaster ? 2.0 : 0.5), 0.125, 4.0);
                std::println("speed x{}", scale_);
                break;
            case Kind::kQuit:
                done_ = true;
                break;
            case Kind::kNone:
                break;
        }
    }

    // Keeps the command going for key_hold_s, bridging the terminal's key-repeat gaps.
    void hold(ServoCommandType::Request::_command_type_type type)
    {
        if (type != command_type_now_)
        {
            auto request = std::make_shared<ServoCommandType::Request>();
            request->command_type = type;
            command_type_->async_send_request(request);
            command_type_now_ = type;
        }
        active_until_ = now() + rclcpp::Duration::from_seconds(key_hold_s_);
    }

    void publish()
    {
        if (command_type_now_ == ServoCommandType::Request::TWIST)
        {
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = now();
            msg.header.frame_id = tool_frame_active_ ? tool_frame_ : base_frame_;
            const double v = linear_speed_mps_ * scale_;
            const double w = angular_speed_radps_ * scale_;
            msg.twist.linear.x = twist_[0] * v;
            msg.twist.linear.y = twist_[1] * v;
            msg.twist.linear.z = twist_[2] * v;
            msg.twist.angular.x = twist_[3] * w;
            msg.twist.angular.y = twist_[4] * w;
            msg.twist.angular.z = twist_[5] * w;
            twist_pub_->publish(msg);
        }
        else
        {
            control_msgs::msg::JointJog msg;
            msg.header.stamp = now();
            msg.joint_names = {joint_names_.at(joint_)};
            msg.velocities = {joint_direction_ * joint_speed_radps_ * scale_};
            jog_pub_->publish(msg);
        }
    }

    void moveGripper(double position_m)
    {
        if (!gripper_->action_server_is_ready())
        {
            RCLCPP_WARN(get_logger(), "Gripper action server is not up");
            return;
        }
        GripperCommand::Goal goal;
        goal.command.name = {gripper_joint_};
        goal.command.position = {position_m};
        goal.command.effort = {gripper_effort_nm_};
        gripper_->async_send_goal(goal);
    }

    double linear_speed_mps_;
    double angular_speed_radps_;
    double joint_speed_radps_;
    double key_hold_s_;
    std::string base_frame_;
    std::string tool_frame_;
    std::vector<std::string> joint_names_;
    std::string gripper_joint_;
    double gripper_open_m_;
    double gripper_closed_m_;
    double gripper_effort_nm_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
    rclcpp::Client<ServoCommandType>::SharedPtr command_type_;
    rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<double, 6> twist_{};
    std::size_t joint_ = 0;
    double joint_direction_ = 0.0;
    double scale_ = 1.0;
    bool tool_frame_active_ = false;
    bool done_ = false;
    // Servo starts in joint jog mode; -1 forces the first request to set the mode.
    ServoCommandType::Request::_command_type_type command_type_now_ = -1;
    rclcpp::Time active_until_{0, 0, RCL_ROS_TIME};
};

}  // namespace

int main(int argc, char ** argv)
try
{
    rclcpp::init(argc, argv);
    const RawTerminal terminal;
    if (!terminal.active())
    {
        std::println(stderr, "keyboard_jog needs a terminal on stdin: run it with ros2 run.");
        rclcpp::shutdown();
        return 1;
    }
    auto node = std::make_shared<KeyboardJog>();
    std::println("{}", piper_teleop::keyHelp());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->done())
    {
        executor.spin_once(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
    return 0;
}
catch (const std::exception & e)
{
    std::fputs(e.what(), stderr);
    return 1;
}
