#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace piper_skills
{

/** @brief Joint values of one named state, by joint name, radians or metres. */
using JointValues = std::map<std::string, double>;

/**
 * @brief The named states (`group_state`) of one planning group in an SRDF.
 * @param srdf The SRDF's XML text.
 * @param group Planning group, e.g. "arm".
 * @return Joint values by state name; empty when the group has none.
 */
[[nodiscard]] std::map<std::string, JointValues> namedStates(
    std::string_view srdf, std::string_view group);

}  // namespace piper_skills
