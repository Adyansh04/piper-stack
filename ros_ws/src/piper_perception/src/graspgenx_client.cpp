#include "piper_perception/graspgenx_client.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <format>
#include <iterator>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <zmq.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace piper_perception
{

// The wire format is little-endian; both targets (x86-64, ARM64) are, so values copy as they are.
static_assert(std::endian::native == std::endian::little);

namespace
{

std::uint32_t readU32(std::span<const std::byte> bytes, std::size_t offset)
{
    std::uint32_t value = 0;
    std::memcpy(&value, bytes.data() + offset, sizeof(value));
    return value;
}

}  // namespace

std::array<std::byte, 8> encodeGraspRequestHeader(std::uint32_t num_grasps)
{
    std::array<std::byte, 8> header{};
    std::memcpy(header.data(), &kGraspProtocolVersion, sizeof(std::uint32_t));
    std::memcpy(header.data() + sizeof(std::uint32_t), &num_grasps, sizeof(std::uint32_t));
    return header;
}

std::expected<GraspReply, std::string> decodeGraspReply(
    std::span<const std::byte> header, std::string_view message, std::span<const std::byte> rows)
{
    if (header.size() != 2 * sizeof(std::uint32_t))
    {
        return std::unexpected(std::format("a {}-byte reply header, 8 expected", header.size()));
    }
    if (const std::uint32_t version = readU32(header, 0); version != kGraspProtocolVersion)
    {
        return std::unexpected(
            std::format(
                "protocol version {}, this client speaks {}", version, kGraspProtocolVersion));
    }
    if (rows.size() % kGraspRowBytes != 0)
    {
        return std::unexpected(
            std::format("{} bytes of grasps: not {}-byte rows", rows.size(), kGraspRowBytes));
    }
    GraspReply reply{
        .success = readU32(header, 4) != 0, .message = std::string(message), .grasps = {}};
    if (!reply.success)
    {
        return reply;
    }
    reply.grasps.resize(rows.size() / kGraspRowBytes);
    std::array<double, 10> row{};
    for (std::size_t i = 0; i < reply.grasps.size(); ++i)
    {
        std::memcpy(row.data(), rows.data() + i * kGraspRowBytes, kGraspRowBytes);
        GraspCandidate & grasp = reply.grasps[i];
        grasp.position_m = {row[0], row[1], row[2]};
        grasp.orientation_xyzw = {row[3], row[4], row[5], row[6]};
        grasp.score = row[7];
        grasp.width_m = row[8];
        grasp.object_width_m = row[9];
    }
    return reply;
}

GraspGenXClient::GraspGenXClient(std::string endpoint) : endpoint_(std::move(endpoint))
{
    connect();
}

void GraspGenXClient::connect()
{
    socket_ = zmq::socket_t(context_, zmq::socket_type::req);
    // Closing must not wait for a request the server never took.
    socket_.set(zmq::sockopt::linger, 0);
    socket_.connect(endpoint_);
}

std::expected<GraspReply, std::string> GraspGenXClient::request(
    std::span<const float> xyz_m, std::uint32_t num_grasps, std::chrono::milliseconds timeout)
{
    if (xyz_m.size() % 3 != 0)
    {
        return std::unexpected(std::format("{} coordinates: not x, y, z triples", xyz_m.size()));
    }
    std::vector<zmq::message_t> frames;
    try
    {
        const auto header = encodeGraspRequestHeader(num_grasps);
        socket_.send(zmq::buffer(header), zmq::send_flags::sndmore);
        socket_.send(zmq::buffer(xyz_m.data(), xyz_m.size_bytes()), zmq::send_flags::none);
        zmq::pollitem_t item{
            .socket = socket_.handle(), .fd = 0, .events = ZMQ_POLLIN, .revents = 0};
        if (zmq::poll(&item, 1, timeout) == 0)
        {
            connect();
            return std::unexpected(
                std::format("no reply from {} in {} ms", endpoint_, timeout.count()));
        }
        (void)zmq::recv_multipart(socket_, std::back_inserter(frames));
    }
    catch (const zmq::error_t & error)
    {
        connect();
        return std::unexpected(std::format("ZMQ error with {}: {}", endpoint_, error.what()));
    }
    if (frames.size() != 3)
    {
        return std::unexpected(std::format("a reply of {} frames, 3 expected", frames.size()));
    }
    const auto bytes = [](const zmq::message_t & frame) {
        return std::span(static_cast<const std::byte *>(frame.data()), frame.size());
    };
    return decodeGraspReply(bytes(frames[0]), frames[1].to_string_view(), bytes(frames[2]));
}

}  // namespace piper_perception
