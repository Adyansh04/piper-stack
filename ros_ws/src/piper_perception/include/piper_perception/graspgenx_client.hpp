#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <zmq.hpp>

namespace piper_perception
{

/**
 * @brief Protocol version spoken with the GraspGenX server (graspgenx_server.py PROTOCOL_VERSION).
 */
inline constexpr std::uint32_t kGraspProtocolVersion = 1;

/**
 * @brief Bytes of one grasp in a reply: ten float64 values.
 */
inline constexpr std::size_t kGraspRowBytes = 10 * sizeof(double);

/**
 * @brief One grasp of a GraspGenX reply.
 */
struct GraspCandidate
{
    /**
     * @brief TCP position in the cloud's frame, metres.
     */
    std::array<double, 3> position_m{};

    /**
     * @brief TCP orientation in the cloud's frame, as a quaternion x, y, z, w: z along the
     *        approach, y between the fingers.
     */
    std::array<double, 4> orientation_xyzw{};

    /**
     * @brief The discriminator's score, 0 to 1.
     */
    double score{0.0};

    /**
     * @brief Finger opening the grasp needs, metres.
     */
    double width_m{0.0};

    /**
     * @brief Width of the object between the fingers, metres.
     */
    double object_width_m{0.0};
};

/**
 * @brief A GraspGenX server's answer to one request.
 */
struct GraspReply
{
    /**
     * @brief Whether the server found grasps.
     */
    bool success{false};

    /**
     * @brief What happened, in words for the log and the operator.
     */
    std::string message;

    /**
     * @brief The grasps, best first; empty when success is false.
     */
    std::vector<GraspCandidate> grasps;
};

/**
 * @brief Encodes the header frame of a request.
 *
 * @param[in] num_grasps Grasps to sample; 0 asks for the server's configured number.
 *
 * @return The protocol version and num_grasps, little-endian uint32 each.
 */
[[nodiscard]] std::array<std::byte, 8> encodeGraspRequestHeader(std::uint32_t num_grasps);

/**
 * @brief Decodes the three frames of a reply.
 *
 * @param[in] header Protocol version and success, little-endian uint32 each.
 * @param[in] message The server's message, UTF-8.
 * @param[in] rows The grasps, kGraspRowBytes each.
 *
 * @return The reply, or why the frames do not follow the protocol.
 */
[[nodiscard]] std::expected<GraspReply, std::string> decodeGraspReply(
    std::span<const std::byte> header, std::string_view message, std::span<const std::byte> rows);

/**
 * @class GraspGenXClient
 * @brief Asks the GraspGenX server for grasps over a ZMQ REQ socket.
 * @details
 * Connects at construction, without waiting for the server: requests sent before it is up wait
 * in the socket. A request that times out closes and reopens the socket, since a REQ socket
 * cannot send again before its reply; a late reply to it is dropped.
 *
 * @note Not thread-safe: one thread uses an instance.
 */
class GraspGenXClient
{
public:
    /**
     * @brief Creates the client and connects to the server.
     *
     * @param[in] endpoint ZMQ endpoint of the server, for example `tcp://127.0.0.1:5561`.
     *
     * @throws zmq::error_t If the endpoint is malformed.
     */
    explicit GraspGenXClient(std::string endpoint);

    /**
     * @brief Sends one object's points and waits for the grasps.
     *
     * @param[in] xyz_m The points, x, y, z per point, metres, in a frame whose z points up.
     * @param[in] num_grasps Grasps to sample; 0 asks for the server's configured number.
     * @param[in] timeout Longest wait for the reply.
     *
     * @return The server's reply (which may report a failure), or why there is none: a timeout or
     *         a reply that does not follow the protocol.
     */
    [[nodiscard]] std::expected<GraspReply, std::string> request(
        std::span<const float> xyz_m, std::uint32_t num_grasps, std::chrono::milliseconds timeout);

private:
    void connect();

    std::string endpoint_;
    zmq::context_t context_;
    zmq::socket_t socket_;
};

}  // namespace piper_perception
