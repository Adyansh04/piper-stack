#pragma once

#include <array>
#include <cstdint>

namespace piper_teleop
{

/**
 * @brief What one key of the keyboard jog asks for.
 */
struct KeyAction
{
    /**
     * @brief The kind of request.
     */
    enum class Kind : std::uint8_t
    {
        kNone,          ///< A key with no meaning.
        kTwist,         ///< Move the tool along or about an axis of the twist frame.
        kSelectJoint,   ///< Choose the joint the jog keys move.
        kJointJog,      ///< Move the selected joint.
        kGripperOpen,   ///< Open the gripper.
        kGripperClose,  ///< Close the gripper.
        kToggleFrame,   ///< Switch the twist frame between the base and the tool.
        kFaster,        ///< Double the speeds.
        kSlower,        ///< Halve the speeds.
        kQuit,          ///< End the node.
    };

    Kind kind = Kind::kNone;
    /// Unit direction for kTwist: linear x, y, z, then angular x, y, z.
    std::array<double, 6> twist{};
    /// The joint for kSelectJoint, 1 to 6.
    int joint = 0;
    /// The direction for kJointJog, +1 or -1.
    double direction = 0.0;
};

/**
 * @brief Decodes one key of the keyboard jog's layout.
 * @param[in] key The character read from the terminal.
 * @return The action, kNone for a key with no meaning.
 */
[[nodiscard]] KeyAction keyAction(char key) noexcept;

/**
 * @brief The layout as help text for the terminal.
 * @return A multi-line description of every key.
 */
[[nodiscard]] const char * keyHelp() noexcept;

}  // namespace piper_teleop
