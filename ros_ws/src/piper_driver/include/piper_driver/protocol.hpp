// Frame layouts follow AgileX's piper_sdk protocol V2 (MIT), for arm firmware before 1.8-8.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <span>

namespace piper_driver::protocol
{

/// @brief Number of arm joints, excluding the gripper.
inline constexpr std::size_t kJointCount = 6;

/// @brief One classic CAN frame with an 11-bit identifier.
struct Frame
{
    std::uint32_t id = 0;                ///< Standard 11-bit identifier.
    std::uint8_t length = 8;             ///< Payload length, 0 to 8 bytes.
    std::array<std::uint8_t, 8> data{};  ///< Payload, big-endian fields.
};

/// @brief CAN identifiers of protocol V2.
namespace id
{
inline constexpr std::uint32_t kMotionCtrl1 = 0x150;      ///< Emergency stop and resume.
inline constexpr std::uint32_t kMotionCtrl2 = 0x151;      ///< Control mode, move mode, speed.
inline constexpr std::uint32_t kJointCtrl12 = 0x155;      ///< Joint 1 and 2 targets.
inline constexpr std::uint32_t kJointCtrl34 = 0x156;      ///< Joint 3 and 4 targets.
inline constexpr std::uint32_t kJointCtrl56 = 0x157;      ///< Joint 5 and 6 targets.
inline constexpr std::uint32_t kGripperCtrl = 0x159;      ///< Gripper target.
inline constexpr std::uint32_t kMotorEnable = 0x471;      ///< Motor enable and disable.
inline constexpr std::uint32_t kLimitQuery = 0x472;       ///< Joint limit query.
inline constexpr std::uint32_t kLimitAngleSpeed = 0x473;  ///< Reply: angle limits, max speed.
inline constexpr std::uint32_t kLimitAccel = 0x47C;       ///< Reply: max acceleration.
inline constexpr std::uint32_t kFirmware = 0x4AF;         ///< Firmware query and its text reply.
inline constexpr std::uint32_t kHighSpeed1 = 0x251;       ///< Motor 1 speed, current, position.
inline constexpr std::uint32_t kHighSpeed6 = 0x256;       ///< Motor 6 speed, current, position.
inline constexpr std::uint32_t kLowSpeed1 = 0x261;  ///< Driver 1 voltage, temperatures, status.
inline constexpr std::uint32_t kLowSpeed6 = 0x266;  ///< Driver 6 voltage, temperatures, status.
inline constexpr std::uint32_t kStatus = 0x2A1;     ///< Arm control mode, status, errors.
inline constexpr std::uint32_t kJoint12 = 0x2A5;    ///< Joint 1 and 2 positions.
inline constexpr std::uint32_t kJoint34 = 0x2A6;    ///< Joint 3 and 4 positions.
inline constexpr std::uint32_t kJoint56 = 0x2A7;    ///< Joint 5 and 6 positions.
inline constexpr std::uint32_t kGripper = 0x2A8;    ///< Gripper stroke, torque, status.
}  // namespace id

/// @brief Whether a frame identifier commands the arm, as opposed to reporting or querying.
/// @param can_id Standard identifier.
/// @return True for motion, joint, gripper, MIT, light and configuration commands.
[[nodiscard]] constexpr bool isCommand(std::uint32_t can_id) noexcept
{
    if (can_id >= 0x150 && can_id <= 0x15F)
    {
        return true;
    }
    switch (can_id)
    {
        case 0x121:
        case 0x470:
        case 0x471:
        case 0x474:
        case 0x475:
        case 0x477:
        case 0x479:
        case 0x47D:
            return true;
        default:
            return false;
    }
}

/// @brief Arm control modes (0x151 byte 0, 0x2A1 byte 0).
enum class CtrlMode : std::uint8_t
{
    kStandby = 0x00,
    kCan = 0x01,
    kTeaching = 0x02
};

/// @brief Firmware motion modes (0x151 byte 1, 0x2A1 byte 2).
enum class MoveMode : std::uint8_t
{
    kP = 0x00,    ///< Cartesian point to point.
    kJ = 0x01,    ///< Joint space, interpolated by the firmware at the commanded speed.
    kL = 0x02,    ///< Linear.
    kC = 0x03,    ///< Circular.
    kMit = 0x04,  ///< Per-joint impedance.
    kCpv = 0x05,  ///< Continuous position and velocity (firmware 1.8-1 and newer).
};

/// @brief Arm status codes reported in 0x2A1 byte 1.
enum class ArmStatus : std::uint8_t
{
    kNormal = 0x00,
    kEmergencyStop = 0x01,
    kNoSolution = 0x02,
    kSingularity = 0x03,
    kTargetBeyondLimit = 0x04,
    kJointCommunication = 0x05,
    kBrakeNotReleased = 0x06,
    kCollision = 0x07,
    kTeachOverspeed = 0x08,
    kJointFault = 0x09,
    kOther = 0x0A,
};

/// @brief Decoded 0x2A1 frame.
struct StatusFeedback
{
    std::uint8_t ctrl_mode = 0;           ///< CtrlMode value.
    std::uint8_t arm_status = 0;          ///< ArmStatus value.
    std::uint8_t move_mode = 0;           ///< MoveMode value in use.
    std::uint8_t teach_status = 0;        ///< Teaching state.
    std::uint8_t motion_status = 0;       ///< 0 when the target is reached, 1 while moving.
    std::uint8_t joint_beyond_limit = 0;  ///< Bit n set: joint n+1 is outside its angle limit.
    std::uint8_t joint_comm_error = 0;    ///< Bit n set: joint n+1 does not communicate.
};

/// @brief Decoded high-speed motor frame (0x251 to 0x256).
struct MotorFeedback
{
    double velocity_rad_s = 0.0;  ///< Motor velocity.
    double current_a = 0.0;       ///< Motor current.
    double effort_nm = 0.0;       ///< Torque from the current, by AgileX's fixed coefficients.
};

/// @brief Decoded low-speed driver frame (0x261 to 0x266).
struct DriverFeedback
{
    double voltage_v = 0.0;          ///< Driver supply voltage.
    std::int16_t driver_temp_c = 0;  ///< Driver temperature.
    std::int8_t motor_temp_c = 0;    ///< Motor temperature.
    std::uint8_t status_bits = 0;    ///< Bit 6 set: motor enabled; other bits are faults.
};

/// @brief Decoded gripper frame (0x2A8).
struct GripperFeedback
{
    double stroke_m = 0.0;         ///< Opening between the fingers.
    double effort_nm = 0.0;        ///< Gripper motor torque.
    std::uint8_t status_bits = 0;  ///< Bit 6 set: enabled; bits 0 to 5 are faults.
};

/// @brief Driver status bit set while a motor is enabled.
inline constexpr std::uint8_t kDriverEnabledBit = 0x40;

// Scales of the wire format.
inline constexpr double kRadPerMilliDeg = std::numbers::pi / 180'000.0;
inline constexpr double kMilliDegPerRad = 180'000.0 / std::numbers::pi;
inline constexpr double kMetrePerMicrometre = 1e-6;
inline constexpr double kMaxGripperEffortNm = 5.0;
// AgileX's current-to-torque coefficients: joints 1 to 3, then 4 to 6.
inline constexpr double kTorquePerAmpBase = 1.18125;
inline constexpr double kTorquePerAmpWrist = 0.95844;

namespace detail
{
constexpr std::int32_t beI32(std::span<const std::uint8_t> b, std::size_t at) noexcept
{
    return static_cast<std::int32_t>(
        (std::uint32_t{b[at]} << 24) | (std::uint32_t{b[at + 1]} << 16)
        | (std::uint32_t{b[at + 2]} << 8) | std::uint32_t{b[at + 3]});
}
constexpr std::int16_t beI16(std::span<const std::uint8_t> b, std::size_t at) noexcept
{
    return static_cast<std::int16_t>((b[at] << 8) | b[at + 1]);
}
constexpr std::uint16_t beU16(std::span<const std::uint8_t> b, std::size_t at) noexcept
{
    return static_cast<std::uint16_t>((b[at] << 8) | b[at + 1]);
}
constexpr void putI32(std::array<std::uint8_t, 8> & b, std::size_t at, std::int32_t v) noexcept
{
    const auto u = static_cast<std::uint32_t>(v);
    b[at] = static_cast<std::uint8_t>(u >> 24);
    b[at + 1] = static_cast<std::uint8_t>(u >> 16);
    b[at + 2] = static_cast<std::uint8_t>(u >> 8);
    b[at + 3] = static_cast<std::uint8_t>(u);
}
constexpr void putU16(std::array<std::uint8_t, 8> & b, std::size_t at, std::uint16_t v) noexcept
{
    b[at] = static_cast<std::uint8_t>(v >> 8);
    b[at + 1] = static_cast<std::uint8_t>(v);
}
// Saturates instead of wrapping: a value outside int32 is a bug upstream, not a far-away target.
inline std::int32_t toI32(double v) noexcept
{
    return static_cast<std::int32_t>(std::clamp(std::round(v), -2147483648.0, 2147483647.0));
}
}  // namespace detail

/// @brief Decodes a joint position frame (0x2A5 to 0x2A7).
/// @param frame Received frame.
/// @param position_rad Joint positions; the two joints the frame carries are written.
/// @return True if the frame was a joint position frame.
inline bool decodeJoints(const Frame & frame, std::span<double, kJointCount> position_rad) noexcept
{
    if (frame.id < id::kJoint12 || frame.id > id::kJoint56 || frame.length < 8)
    {
        return false;
    }
    const std::size_t first = 2 * (frame.id - id::kJoint12);
    position_rad[first] = detail::beI32(frame.data, 0) * kRadPerMilliDeg;
    position_rad[first + 1] = detail::beI32(frame.data, 4) * kRadPerMilliDeg;
    return true;
}

/// @brief Decodes the arm status frame (0x2A1).
/// @param frame Received frame.
/// @return The status, or nothing if the frame is another one.
inline std::optional<StatusFeedback> decodeStatus(const Frame & frame) noexcept
{
    if (frame.id != id::kStatus || frame.length < 8)
    {
        return std::nullopt;
    }
    const auto & d = frame.data;
    return StatusFeedback{
        .ctrl_mode = d[0],
        .arm_status = d[1],
        .move_mode = d[2],
        .teach_status = d[3],
        .motion_status = d[4],
        .joint_beyond_limit = d[6],
        .joint_comm_error = d[7]};
}

/// @brief Decodes a high-speed motor frame (0x251 to 0x256).
/// @param frame Received frame.
/// @param motor Motor feedback per joint; the frame's joint is written.
/// @return True if the frame was a high-speed motor frame.
inline bool decodeMotor(const Frame & frame, std::span<MotorFeedback, kJointCount> motor) noexcept
{
    if (frame.id < id::kHighSpeed1 || frame.id > id::kHighSpeed6 || frame.length < 8)
    {
        return false;
    }
    const std::size_t joint = frame.id - id::kHighSpeed1;
    const double current_a = detail::beI16(frame.data, 2) * 1e-3;
    motor[joint] = MotorFeedback{
        .velocity_rad_s = detail::beI16(frame.data, 0) * 1e-3,
        .current_a = current_a,
        .effort_nm = current_a * (joint < 3 ? kTorquePerAmpBase : kTorquePerAmpWrist)};
    return true;
}

/// @brief Decodes a low-speed driver frame (0x261 to 0x266).
/// @param frame Received frame.
/// @param driver Driver feedback per joint; the frame's joint is written.
/// @return True if the frame was a low-speed driver frame.
inline bool decodeDriver(
    const Frame & frame, std::span<DriverFeedback, kJointCount> driver) noexcept
{
    if (frame.id < id::kLowSpeed1 || frame.id > id::kLowSpeed6 || frame.length < 8)
    {
        return false;
    }
    driver[frame.id - id::kLowSpeed1] = DriverFeedback{
        .voltage_v = detail::beU16(frame.data, 0) * 0.1,
        .driver_temp_c = detail::beI16(frame.data, 2),
        .motor_temp_c = static_cast<std::int8_t>(frame.data[4]),
        .status_bits = frame.data[5]};
    return true;
}

/// @brief Decodes the gripper frame (0x2A8).
/// @param frame Received frame.
/// @return The gripper feedback, or nothing if the frame is another one.
inline std::optional<GripperFeedback> decodeGripper(const Frame & frame) noexcept
{
    if (frame.id != id::kGripper || frame.length < 8)
    {
        return std::nullopt;
    }
    return GripperFeedback{
        .stroke_m = detail::beI32(frame.data, 0) * kMetrePerMicrometre,
        .effort_nm = detail::beI16(frame.data, 4) * 1e-3,
        .status_bits = frame.data[6]};
}

/// @brief Encodes motor enable or disable for all joints and the gripper (0x471).
/// @param enable True to enable, false to disable.
/// @return The frame.
/// @warning Disabling removes motor torque: a raised arm falls.
constexpr Frame encodeEnableAll(bool enable) noexcept
{
    return Frame{
        .id = id::kMotorEnable,
        .length = 8,
        .data = {0x07, static_cast<std::uint8_t>(enable ? 0x02 : 0x01), 0, 0, 0, 0, 0, 0}};
}

/// @brief Encodes the mode command (0x151).
/// @param ctrl Control mode.
/// @param move Motion mode.
/// @param speed_percent Firmware speed scaling, clamped to 0 to 100.
/// @return The frame, with position-velocity (non-MIT) mode and no installation change.
constexpr Frame encodeMode(CtrlMode ctrl, MoveMode move, int speed_percent) noexcept
{
    return Frame{
        .id = id::kMotionCtrl2,
        .length = 8,
        .data = {
            static_cast<std::uint8_t>(ctrl),
            static_cast<std::uint8_t>(move),
            static_cast<std::uint8_t>(std::clamp(speed_percent, 0, 100)),
            0x00,
            0,
            0,
            0,
            0}};
}

/// @brief Encodes the emergency stop (0x150 byte 0 = 0x01) or its resume (0x02).
/// @param stop True for the emergency stop, false to resume.
/// @return The frame.
constexpr Frame encodeEmergencyStop(bool stop) noexcept
{
    return Frame{
        .id = id::kMotionCtrl1,
        .length = 8,
        .data = {static_cast<std::uint8_t>(stop ? 0x01 : 0x02), 0, 0, 0, 0, 0, 0, 0}};
}

/// @brief Encodes joint targets (0x155 to 0x157).
/// @param position_rad Targets per joint.
/// @return Three frames, joints 1 and 2 first.
inline std::array<Frame, 3> encodeJoints(std::span<const double, kJointCount> position_rad) noexcept
{
    std::array<Frame, 3> frames{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        frames[i].id = id::kJointCtrl12 + static_cast<std::uint32_t>(i);
        detail::putI32(frames[i].data, 0, detail::toI32(position_rad[2 * i] * kMilliDegPerRad));
        detail::putI32(frames[i].data, 4, detail::toI32(position_rad[2 * i + 1] * kMilliDegPerRad));
    }
    return frames;
}

/// @brief Decodes joint targets from 0x155 to 0x157, for the fake arm and bus monitors.
/// @param frame Received frame.
/// @param position_rad Targets; the two joints the frame carries are written.
/// @return True if the frame was a joint target frame.
inline bool decodeJointTargets(
    const Frame & frame, std::span<double, kJointCount> position_rad) noexcept
{
    if (frame.id < id::kJointCtrl12 || frame.id > id::kJointCtrl56 || frame.length < 8)
    {
        return false;
    }
    const std::size_t first = 2 * (frame.id - id::kJointCtrl12);
    position_rad[first] = detail::beI32(frame.data, 0) * kRadPerMilliDeg;
    position_rad[first + 1] = detail::beI32(frame.data, 4) * kRadPerMilliDeg;
    return true;
}

/// @brief Encodes a gripper target (0x159) that keeps the gripper enabled.
/// @param stroke_m Opening between the fingers.
/// @param effort_nm Grip torque, clamped to 0 to 5 N m.
/// @return The frame.
inline Frame encodeGripper(double stroke_m, double effort_nm) noexcept
{
    Frame frame{.id = id::kGripperCtrl, .length = 8, .data = {}};
    detail::putI32(frame.data, 0, detail::toI32(stroke_m / kMetrePerMicrometre));
    detail::putU16(
        frame.data,
        4,
        static_cast<std::uint16_t>(
            std::round(std::clamp(effort_nm, 0.0, kMaxGripperEffortNm) * 1e3)));
    frame.data[6] = 0x01;
    return frame;
}

/// @brief Encodes the firmware version query (0x4AF); the reply is ASCII text on the same id.
constexpr Frame encodeFirmwareQuery() noexcept
{
    return Frame{.id = id::kFirmware, .length = 8, .data = {0x01, 0, 0, 0, 0, 0, 0, 0}};
}

}  // namespace piper_driver::protocol
