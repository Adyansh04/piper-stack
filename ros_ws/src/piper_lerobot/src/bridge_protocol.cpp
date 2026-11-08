#include "piper_lerobot/bridge_protocol.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <format>
#include <span>
#include <string>
#include <vector>

namespace piper_lerobot
{

// The frames carry numbers in the host's order; both targets (x86-64, ARM64) are little-endian.
static_assert(std::endian::native == std::endian::little);

namespace
{

template <typename T>
void append(std::vector<std::byte> & frame, T value)
{
    const auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
    frame.insert(frame.end(), bytes.begin(), bytes.end());
}

}  // namespace

std::vector<std::byte> encodeState(
    double stamp_s, std::span<const double> measured, std::span<const double> commanded)
{
    std::vector<std::byte> frame;
    frame.reserve(sizeof(double) * (1 + measured.size() + commanded.size()));
    append(frame, stamp_s);
    for (const double value : measured)
    {
        append(frame, value);
    }
    for (const double value : commanded)
    {
        append(frame, value);
    }
    return frame;
}

std::vector<std::byte> encodeImageSize(std::uint32_t height, std::uint32_t width)
{
    std::vector<std::byte> frame;
    frame.reserve(2 * sizeof(std::uint32_t));
    append(frame, height);
    append(frame, width);
    return frame;
}

std::expected<std::vector<double>, std::string> decodePositions(
    std::span<const std::byte> frame, std::size_t joints)
{
    if (frame.size() != joints * sizeof(double))
    {
        return std::unexpected(
            std::format(
                "{} bytes of positions, {} expected ({} float64)",
                frame.size(),
                joints * sizeof(double),
                joints));
    }
    std::vector<double> positions(joints);
    std::memcpy(positions.data(), frame.data(), frame.size());
    if (!std::ranges::all_of(positions, [](double value) { return std::isfinite(value); }))
    {
        return std::unexpected("a position is not finite");
    }
    return positions;
}

std::vector<double> jogVelocities(
    std::span<const double> target,
    std::span<const double> measured,
    double gain,
    double max_velocity_rad_s)
{
    const std::size_t joints = std::min(target.size(), measured.size());
    std::vector<double> velocities(joints);
    for (std::size_t i = 0; i < joints; ++i)
    {
        velocities[i] =
            std::clamp(gain * (target[i] - measured[i]), -max_velocity_rad_s, max_velocity_rad_s);
    }
    return velocities;
}

}  // namespace piper_lerobot
