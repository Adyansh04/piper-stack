#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace piper_lerobot
{

/**
 * @brief The frames of the bridge's ZMQ request-reply protocol with LeRobot's plugins.
 * @details Requests: `observe`; `act` and a frame of joint positions; `release`. Replies: `ok`,
 * for `observe` followed by a frame of the state, a frame of the image size and a frame of the
 * image (rgb8, row by row); or `error` and a frame of UTF-8 text. Numbers are little-endian:
 * positions and the state are float64, the image size two uint32 (height, width). The state is the
 * stamp (s), then the measured and the commanded position of each joint (rad, or m for the
 * gripper).
 */
namespace protocol
{
inline constexpr std::string_view kObserve = "observe";
inline constexpr std::string_view kAct = "act";
inline constexpr std::string_view kRelease = "release";
inline constexpr std::string_view kOk = "ok";
inline constexpr std::string_view kError = "error";
}  // namespace protocol

/**
 * @brief Encodes the state frame of an `observe` reply.
 * @param[in] stamp_s Time of the joint readings, s.
 * @param[in] measured Measured position of each joint.
 * @param[in] commanded Commanded position of each joint, the same count.
 * @return The frame: 1 + 2 n float64.
 */
[[nodiscard]] std::vector<std::byte> encodeState(
    double stamp_s, std::span<const double> measured, std::span<const double> commanded);

/**
 * @brief Encodes the image size frame of an `observe` reply.
 * @param[in] height Rows of the image.
 * @param[in] width Columns of the image.
 * @return The frame: two uint32.
 */
[[nodiscard]] std::vector<std::byte> encodeImageSize(std::uint32_t height, std::uint32_t width);

/**
 * @brief Decodes the positions frame of an `act` request.
 * @param[in] frame The frame's bytes.
 * @param[in] joints How many positions the bridge expects.
 * @return The positions, or why the frame is not that: another length, or a non-finite value.
 */
[[nodiscard]] std::expected<std::vector<double>, std::string> decodePositions(
    std::span<const std::byte> frame, std::size_t joints);

/**
 * @brief Joint velocities that bring the joints to target positions, for Servo's joint jog.
 * @param[in] target Target position of each joint, rad.
 * @param[in] measured Measured position of each joint, rad, the same count.
 * @param[in] gain Velocity per unit of position error, 1/s.
 * @param[in] max_velocity_rad_s Largest velocity of a joint, rad/s.
 * @return One velocity per joint, rad/s: the gain times the error, clamped to the cap.
 */
[[nodiscard]] std::vector<double> jogVelocities(
    std::span<const double> target,
    std::span<const double> measured,
    double gain,
    double max_velocity_rad_s);

}  // namespace piper_lerobot
