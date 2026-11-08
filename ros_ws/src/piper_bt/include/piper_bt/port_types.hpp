#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace BT
{

/**
 * @brief A pose written in a tree as "x;y;yaw" or "x;y;yaw;frame".
 * @param str Metres and radians, separated by ';'; spaces around a part are ignored.
 * @return The pose, z 0, rotated by yaw about z. The three-part form leaves the frame empty,
 * which NavigateToPose fills with the executor's map_frame.
 * @throws BT::RuntimeError If there are not three numbers, a number does not parse whole or is
 * not finite, or a fourth part names no frame.
 */
template <>
[[nodiscard]] geometry_msgs::msg::PoseStamped convertFromString(StringView str);

}  // namespace BT
