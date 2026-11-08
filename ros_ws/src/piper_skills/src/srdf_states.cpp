#include "piper_skills/srdf_states.hpp"

#include <tinyxml2.h>

#include <map>
#include <string>
#include <string_view>

namespace piper_skills
{

std::map<std::string, JointValues> namedStates(std::string_view srdf, std::string_view group)
{
    std::map<std::string, JointValues> out;
    tinyxml2::XMLDocument doc;
    // An XML parser, so comments, attribute order and quoting do not matter.
    if (doc.Parse(srdf.data(), srdf.size()) != tinyxml2::XML_SUCCESS
        || doc.RootElement() == nullptr)
    {
        return out;
    }
    for (const tinyxml2::XMLElement * state = doc.RootElement()->FirstChildElement("group_state");
         state != nullptr;
         state = state->NextSiblingElement("group_state"))
    {
        const char * name = state->Attribute("name");
        const char * in_group = state->Attribute("group");
        if (name == nullptr || in_group == nullptr || group != in_group)
        {
            continue;
        }
        JointValues values;
        for (const tinyxml2::XMLElement * joint = state->FirstChildElement("joint");
             joint != nullptr;
             joint = joint->NextSiblingElement("joint"))
        {
            const char * joint_name = joint->Attribute("name");
            double value = 0.0;
            if (joint_name != nullptr
                && joint->QueryDoubleAttribute("value", &value) == tinyxml2::XML_SUCCESS)
            {
                values[joint_name] = value;
            }
        }
        out[name] = values;
    }
    return out;
}

}  // namespace piper_skills
