#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <string>
#include <system_error>

#include "piper_driver/protocol.hpp"

namespace piper_driver
{

/// @brief A frame read from the bus, with the kernel's receive time.
struct ReceivedFrame
{
    protocol::Frame frame;      ///< The frame; meaningless when error is set.
    std::int64_t stamp_ns = 0;  ///< Kernel receive time, CLOCK_REALTIME, in nanoseconds.
    bool error = false;         ///< True for a controller error report (bus-off, error passive).
};

/**
 * @class CanBus
 * @brief A raw SocketCAN socket bound to one interface.
 * @details send() never blocks: a full transmit queue is an error, not a wait. receive() waits up
 * to its timeout. Frames this socket sends are not received back; frames sent by other sockets of
 * this host are, as are frames from other nodes on the bus.
 */
class CanBus
{
public:
    /**
   * @brief Opens a socket on a CAN interface that is up.
   * @param[in] interface Interface name, such as `can0`.
   * @return The bus, or the system error that prevented opening it.
   */
    [[nodiscard]] static std::expected<CanBus, std::error_code> open(const std::string & interface);

    CanBus(CanBus && other) noexcept;
    CanBus & operator=(CanBus && other) noexcept;
    CanBus(const CanBus &) = delete;
    CanBus & operator=(const CanBus &) = delete;
    ~CanBus();

    /**
   * @brief Queues one frame for transmission without blocking.
   * @param[in] frame Frame to send.
   * @return Nothing, or the error (`no_buffer_space` or `resource_unavailable_try_again` when the
   *         transmit queue is full).
   */
    [[nodiscard]] std::expected<void, std::error_code> send(const protocol::Frame & frame) noexcept;

    /**
   * @brief Waits for the next frame.
   * @param[in] timeout How long to wait; zero polls without waiting.
   * @return The frame, `timed_out` if none arrived in time, or another system error.
   */
    [[nodiscard]] std::expected<ReceivedFrame, std::error_code> receive(
        std::chrono::milliseconds timeout) noexcept;

private:
    explicit CanBus(int fd) noexcept : fd_(fd) {}

    int fd_ = -1;
};

}  // namespace piper_driver
