#include "piper_driver/can_bus.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <expected>
#include <iterator>
#include <string>
#include <system_error>
#include <utility>

#include "piper_driver/protocol.hpp"

namespace piper_driver
{

namespace
{
std::error_code lastError() noexcept
{
    return {errno, std::system_category()};
}
}  // namespace

std::expected<CanBus, std::error_code> CanBus::open(const std::string & interface)
{
    if (interface.empty() || interface.size() >= IFNAMSIZ)
    {
        return std::unexpected(std::make_error_code(std::errc::invalid_argument));
    }
    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0)
    {
        return std::unexpected(lastError());
    }
    CanBus bus(fd);
    ifreq ifr{};
    std::memcpy(std::data(ifr.ifr_name), interface.c_str(), interface.size() + 1);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): the index is only reachable through ioctl
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
    {
        return std::unexpected(lastError());
    }
    // Controller problems arrive as error frames, so a bus-off is seen, not mistaken for silence.
    const can_err_mask_t errors = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_RESTARTED;
    const int on = 1;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0
        || ::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on)) < 0)
    {
        return std::unexpected(lastError());
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): bind() takes the generic type
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        return std::unexpected(lastError());
    }
    return bus;
}

CanBus::CanBus(CanBus && other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

CanBus & CanBus::operator=(CanBus && other) noexcept
{
    if (this != &other)
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
        fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
}

CanBus::~CanBus()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
    }
}

std::expected<void, std::error_code> CanBus::send(const protocol::Frame & frame) const noexcept
{
    can_frame raw{};
    raw.can_id = frame.id & CAN_SFF_MASK;
    raw.len = frame.length;  // NOLINT(cppcoreguidelines-pro-type-union-access): kernel ABI
    std::memcpy(std::data(raw.data), frame.data.data(), sizeof(raw.data));
    if (::write(fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw)))
    {
        return std::unexpected(lastError());
    }
    return {};
}

std::expected<ReceivedFrame, std::error_code> CanBus::receive(
    std::chrono::milliseconds timeout) noexcept
{
    pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
    if (ready < 0)
    {
        return std::unexpected(lastError());
    }
    if (ready == 0)
    {
        return std::unexpected(std::make_error_code(std::errc::timed_out));
    }
    can_frame raw{};
    iovec iov{.iov_base = &raw, .iov_len = sizeof(raw)};
    alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))> control{};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data();
    msg.msg_controllen = control.size();
    if (::recvmsg(fd_, &msg, 0) != static_cast<ssize_t>(sizeof(raw)))
    {
        return std::unexpected(lastError());
    }
    ReceivedFrame out;
    for (cmsghdr * c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c))
    {
        if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS)
        {
            timespec ts{};
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            out.stamp_ns = std::int64_t{ts.tv_sec} * 1'000'000'000 + ts.tv_nsec;
        }
    }
    out.error = (raw.can_id & CAN_ERR_FLAG) != 0;
    out.frame.id = raw.can_id & CAN_SFF_MASK;
    out.frame.length = raw.len;  // NOLINT(cppcoreguidelines-pro-type-union-access): kernel ABI
    std::memcpy(out.frame.data.data(), std::data(raw.data), sizeof(raw.data));
    return out;
}

}  // namespace piper_driver
