#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <set>
#include <string_view>
#include <utility>

#include "piper_teleop/key_map.hpp"

namespace
{

using piper_teleop::KeyAction;
using piper_teleop::keyAction;

TEST(KeyMap, TwistKeysCoverEveryAxisBothWaysOnce)
{
    std::set<std::pair<int, double>> seen;
    for (const char key : std::string_view("wsadrfuoikjl"))
    {
        const KeyAction action = keyAction(key);
        ASSERT_EQ(action.kind, KeyAction::Kind::kTwist) << key;
        int axes = 0;
        for (int axis = 0; axis < 6; ++axis)
        {
            const double v = action.twist.at(static_cast<std::size_t>(axis));
            if (v != 0.0)
            {
                ++axes;
                EXPECT_TRUE(seen.emplace(axis, v).second) << key << " repeats an axis and sign";
            }
        }
        EXPECT_EQ(axes, 1) << key;
    }
    EXPECT_EQ(seen.size(), 12U);
}

TEST(KeyMap, JointKeysSelectAndJog)
{
    EXPECT_EQ(keyAction('1').kind, KeyAction::Kind::kSelectJoint);
    EXPECT_EQ(keyAction('6').joint, 6);
    EXPECT_EQ(keyAction('7').kind, KeyAction::Kind::kNone);
    EXPECT_DOUBLE_EQ(keyAction('+').direction, 1.0);
    EXPECT_DOUBLE_EQ(keyAction('=').direction, 1.0);
    EXPECT_DOUBLE_EQ(keyAction('-').direction, -1.0);
}

TEST(KeyMap, UnknownKeysDoNothing)
{
    for (const char key : std::string_view("xzZ \n\x1b"))
    {
        EXPECT_EQ(keyAction(key).kind, KeyAction::Kind::kNone) << static_cast<int>(key);
    }
}

}  // namespace
