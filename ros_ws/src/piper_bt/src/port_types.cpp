#include "piper_bt/port_types.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>

#include <cmath>
#include <format>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace BT
{

namespace
{

StringView trimmed(StringView s)
{
    const auto first = s.find_first_not_of(' ');
    if (first == StringView::npos)
    {
        return {};
    }
    return s.substr(first, s.find_last_not_of(' ') - first + 1);
}

}  // namespace

template <>
geometry_msgs::msg::PoseStamped convertFromString(StringView str)
{
    const auto parts = splitString(str, ';');
    if (parts.size() != 3 && parts.size() != 4)
    {
        throw RuntimeError(std::format("a pose is 'x;y;yaw' or 'x;y;yaw;frame', got '{}'", str));
    }
    // A decimal comma or a unit would otherwise end a number early: "1,5" read as 1.
    const auto number = [&](StringView part) {
        double value = 0.0;
        if (!parseDouble(trimmed(part), value, true) || !std::isfinite(value))
        {
            throw RuntimeError(std::format("'{}' in the pose '{}' is not a number", part, str));
        }
        return value;
    };
    geometry_msgs::msg::PoseStamped pose;
    if (parts.size() == 4)
    {
        pose.header.frame_id = std::string(trimmed(parts[3]));
        if (pose.header.frame_id.empty())
        {
            throw RuntimeError(std::format("the pose '{}' names no frame", str));
        }
    }
    pose.pose.position.x = number(parts[0]);
    pose.pose.position.y = number(parts[1]);
    const double yaw_rad = number(parts[2]);
    pose.pose.orientation.z = std::sin(0.5 * yaw_rad);
    pose.pose.orientation.w = std::cos(0.5 * yaw_rad);
    return pose;
}

}  // namespace BT
