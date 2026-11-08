#include "piper_teleop/key_map.hpp"

#include <array>
#include <cstddef>

namespace piper_teleop
{

namespace
{

KeyAction twist(int axis, double sign) noexcept
{
    KeyAction action{.kind = KeyAction::Kind::kTwist};
    action.twist.at(static_cast<std::size_t>(axis)) = sign;
    return action;
}

}  // namespace

KeyAction keyAction(char key) noexcept
{
    using Kind = KeyAction::Kind;
    switch (key)
    {
        case 'w':
            return twist(0, 1.0);
        case 's':
            return twist(0, -1.0);
        case 'a':
            return twist(1, 1.0);
        case 'd':
            return twist(1, -1.0);
        case 'r':
            return twist(2, 1.0);
        case 'f':
            return twist(2, -1.0);
        case 'u':
            return twist(3, 1.0);
        case 'o':
            return twist(3, -1.0);
        case 'i':
            return twist(4, 1.0);
        case 'k':
            return twist(4, -1.0);
        case 'j':
            return twist(5, 1.0);
        case 'l':
            return twist(5, -1.0);
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
            return {.kind = Kind::kSelectJoint, .joint = key - '0'};
        case '+':
        case '=':
            return {.kind = Kind::kJointJog, .direction = 1.0};
        case '-':
            return {.kind = Kind::kJointJog, .direction = -1.0};
        case 'g':
            return {.kind = Kind::kGripperOpen};
        case 'h':
            return {.kind = Kind::kGripperClose};
        case 't':
            return {.kind = Kind::kToggleFrame};
        case ']':
            return {.kind = Kind::kFaster};
        case '[':
            return {.kind = Kind::kSlower};
        case 'q':
            return {.kind = Kind::kQuit};
        default:
            return {};
    }
}

const char * keyHelp() noexcept
{
    return R"(Keyboard jog of the Piper through MoveIt Servo. Hold a key to move.
  w/s  +x/-x     a/d  +y/-y     r/f  +z/-z       (along the twist frame's axes)
  u/o  roll      i/k  pitch     j/l  yaw         (about them)
  1-6  choose a joint, then +/- to jog it
  g/h  open/close the gripper
  t    twist frame: base_link <-> tcp
  [ ]  halve / double the speeds
  q    quit)";
}

}  // namespace piper_teleop
