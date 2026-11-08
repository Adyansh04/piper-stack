#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <system_error>
#include <thread>

#include "piper_driver/can_bus.hpp"
#include "piper_driver/protocol.hpp"
#include "piper_driver/triple_buffer.hpp"

namespace piper_driver
{

/// @brief Everything the arm reports, as of the newest frame.
struct ArmState
{
    std::array<double, protocol::kJointCount> position_rad{};            ///< Joint positions.
    std::array<protocol::MotorFeedback, protocol::kJointCount> motor{};  ///< Per-joint motor data.
    std::array<protocol::DriverFeedback, protocol::kJointCount> driver{};  ///< Per-joint drivers.
    protocol::GripperFeedback gripper{};                                   ///< Gripper.
    protocol::StatusFeedback status{};  ///< Arm control mode, status and error bits.
    std::uint64_t joint_frames = 0;     ///< Joint position frames received; grows with each one.
    std::int64_t joint_stamp_ns = 0;    ///< Kernel receive time of the newest joint frame.
    std::uint64_t frames = 0;           ///< All frames received.
    std::uint64_t foreign_commands =
        0;                         ///< Command frames on the bus that this driver did not send.
    std::uint64_t bus_errors = 0;  ///< Controller error reports (bus-off, error passive).
};

/**
 * @class Arm
 * @brief The Piper on one CAN interface: its feedback, decoded continuously, and its commands.
 * @details A receive thread decodes every frame into an ArmState. latest() returns the newest state
 * without blocking. Commands are sent at once, without blocking, and validated first.
 *
 * Construction and destruction send nothing: the arm keeps whatever state it is in.
 *
 * @warning latest() and the command functions may each be called from one thread only (the
 *          control loop). enable(false) removes motor torque: a raised arm falls.
 */
// One per process, so its padding costs nothing; receiver_ must stay the last member.
class Arm  // NOLINT(clang-analyzer-optin.performance.Padding)
{
public:
    /**
   * @brief Opens the interface and starts decoding feedback.
   * @param[in] interface CAN interface the arm is on, up at 1 Mbit/s.
   * @return The arm, or the error from opening the interface.
   */
    [[nodiscard]] static std::expected<std::unique_ptr<Arm>, std::error_code> open(
        const std::string & interface);

    Arm(const Arm &) = delete;
    Arm & operator=(const Arm &) = delete;
    ~Arm();

    /**
   * @brief Returns the newest state without blocking.
   * @return A reference valid until the next call.
   */
    [[nodiscard]] const ArmState & latest() noexcept
    {
        return state_.read();
    }

    /**
   * @brief Enables or disables all joint motors and the gripper.
   * @param[in] on True to enable.
   * @return Nothing, or the send error.
   * @warning Disabling removes torque at once: a raised arm falls.
   */
    std::expected<void, std::error_code> enable(bool on) noexcept;

    /**
   * @brief Sets the control and motion mode.
   * @param[in] ctrl Control mode; kCan accepts commands from this driver.
   * @param[in] move Motion mode.
   * @param[in] speed_percent Firmware speed scaling, 0 to 100.
   * @return Nothing, or the send error.
   */
    std::expected<void, std::error_code> setMode(
        protocol::CtrlMode ctrl, protocol::MoveMode move, int speed_percent) noexcept;

    /**
   * @brief Sends joint targets.
   * @param[in] position_rad Target per joint.
   * @return Nothing, `invalid_argument` if a target is not finite, or the send error.
   */
    std::expected<void, std::error_code> commandJoints(
        std::span<const double, protocol::kJointCount> position_rad) noexcept;

    /**
   * @brief Sends a gripper target.
   * @param[in] stroke_m Opening between the fingers, in metres.
   * @param[in] effort_nm Grip torque, 0 to 5 N m.
   * @return Nothing, `invalid_argument` if a value is not finite, or the send error.
   */
    std::expected<void, std::error_code> commandGripper(double stroke_m, double effort_nm) noexcept;

    /**
   * @brief Sends the firmware's emergency stop, or its resume.
   * @param[in] stop True to stop, false to resume.
   * @return Nothing, or the send error.
   */
    std::expected<void, std::error_code> emergencyStop(bool stop) noexcept;

    /**
   * @brief Asks the arm for its firmware text; firmwareText() holds the reply once it arrives.
   * @return Nothing, or the send error.
   */
    std::expected<void, std::error_code> requestFirmware() noexcept;

    /**
   * @brief Returns the firmware text received so far, such as `S-V1.8-2`.
   * @return The version field, or an empty string before any reply.
   */
    [[nodiscard]] std::string firmwareText() const;

private:
    explicit Arm(CanBus bus);
    void receiveLoop(const std::stop_token & stop);
    std::expected<void, std::error_code> send(const protocol::Frame & frame) noexcept;

    CanBus bus_;
    TripleBuffer<ArmState> state_;
    mutable std::mutex firmware_mutex_;
    std::string firmware_raw_;  // Guarded by firmware_mutex_.
    std::jthread receiver_;     // Last: it stops and joins before the members it uses go.
};

}  // namespace piper_driver
