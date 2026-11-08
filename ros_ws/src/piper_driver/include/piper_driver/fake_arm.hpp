#pragma once

#include <array>
#include <chrono>
#include <expected>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>

#include "piper_driver/can_bus.hpp"
#include "piper_driver/protocol.hpp"

namespace piper_driver
{

/**
 * @class FakeArm
 * @brief A stand-in for the Piper's firmware on a virtual CAN interface, for tests without the arm.
 * @details Sends the arm's feedback frames every period and obeys enable, mode, emergency stop,
 * joint, gripper and firmware-query frames. Enabled in CAN control mode with MOVE J, each joint
 * moves toward its target at the maximum speed scaled by the commanded speed percent; anything
 * else holds the joints still.
 *
 * @note It models the protocol, not the arm's dynamics or every firmware rule.
 */
class FakeArm
{
public:
    /// @brief Start-up settings.
    struct Options
    {
        std::string interface = "vcan0";         ///< Virtual CAN interface, up.
        std::chrono::microseconds period{5000};  ///< Feedback period: 200 Hz like the arm.
        double max_speed_rad_s = 3.0;            ///< Joint speed at 100 percent.
        std::array<double, protocol::kJointCount> start_rad{};  ///< Initial joint positions.
    };

    /**
   * @brief Opens the interface and starts the fake firmware.
   * @param[in] options Interface, period and initial pose.
   * @return The running fake arm, or the error from opening the interface.
   */
    [[nodiscard]] static std::expected<std::unique_ptr<FakeArm>, std::error_code> start(
        const Options & options);

    FakeArm(const FakeArm &) = delete;
    FakeArm & operator=(const FakeArm &) = delete;
    ~FakeArm();

    /// @brief Current joint positions, in radians.
    [[nodiscard]] std::array<double, protocol::kJointCount> positions() const;

    /// @brief Whether the motors are enabled.
    [[nodiscard]] bool enabled() const;

    /// @brief Returns the collision levels last set over the bus.
    [[nodiscard]] std::array<std::uint8_t, protocol::kJointCount> collisionLevels() const;

    /// @brief Returns the end load last set over the bus (0x477 byte 4).
    [[nodiscard]] std::uint8_t endLoad() const;

private:
    explicit FakeArm(CanBus bus, const Options & options);
    void run(const std::stop_token & stop);
    void apply(const protocol::Frame & frame);
    void step(double dt_s);
    void sendFeedback(std::uint64_t cycle);

    CanBus bus_;
    Options options_;
    mutable std::mutex mutex_;  // Guards everything below; the run thread and the getters use it.
    std::array<double, protocol::kJointCount> position_rad_{};
    std::array<double, protocol::kJointCount> target_rad_{};
    double gripper_stroke_m_ = 0.0;
    bool enabled_ = false;
    bool stopped_ = false;
    std::uint8_t ctrl_mode_ = 0;
    std::uint8_t move_mode_ = 0;
    bool mit_ = false;
    std::array<std::uint8_t, protocol::kJointCount> collision_levels_{};
    std::uint8_t end_load_ = 0;
    int speed_percent_ = 0;
    std::jthread runner_;  // Last: it stops and joins before the members it uses go.
};

}  // namespace piper_driver
