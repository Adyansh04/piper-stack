#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <span>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <zmq.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "piper_perception/graspgenx_client.hpp"

namespace piper_perception
{
namespace
{

using std::chrono::milliseconds;

// Points sent per request in the client tests.
constexpr std::size_t kPoints = 50;

std::vector<std::byte> header(std::uint32_t version, std::uint32_t second)
{
    std::vector<std::byte> out(8);
    std::memcpy(out.data(), &version, 4);
    std::memcpy(out.data() + 4, &second, 4);
    return out;
}

std::vector<std::byte> rows(const std::vector<std::array<double, 10>> & values)
{
    std::vector<std::byte> out(values.size() * kGraspRowBytes);
    std::memcpy(out.data(), values.data(), out.size());
    return out;
}

/**
 * @brief A REP server on a free local port that answers each request with a canned reply,
 *        after skipping the first `ignore` requests (it replies to those late).
 */
class FakeServer
{
public:
    explicit FakeServer(int ignore = 0) : socket_(context_, zmq::socket_type::rep)
    {
        socket_.bind("tcp://127.0.0.1:*");
        endpoint_ = socket_.get(zmq::sockopt::last_endpoint);
        thread_ = std::jthread([this, ignore](const std::stop_token & stop) {
            int seen = 0;
            while (!stop.stop_requested())
            {
                zmq::pollitem_t item{
                    .socket = socket_.handle(), .fd = 0, .events = ZMQ_POLLIN, .revents = 0};
                if (zmq::poll(&item, 1, milliseconds(20)) == 0)
                {
                    continue;
                }
                std::vector<zmq::message_t> request;
                (void)zmq::recv_multipart(socket_, std::back_inserter(request));
                points_ = request.size() == 2 ? request[1].size() / (3 * sizeof(float)) : 0;
                if (seen++ < ignore)
                {
                    std::this_thread::sleep_for(milliseconds(300));
                }
                const auto head = header(kGraspProtocolVersion, 1);
                const auto body = rows({{0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 0.9, 0.05, 0.04}});
                socket_.send(zmq::buffer(head), zmq::send_flags::sndmore);
                socket_.send(zmq::str_buffer("1 grasps"), zmq::send_flags::sndmore);
                socket_.send(zmq::buffer(body), zmq::send_flags::none);
            }
        });
    }

    [[nodiscard]] const std::string & endpoint() const
    {
        return endpoint_;
    }
    [[nodiscard]] std::size_t points() const
    {
        return points_;
    }

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    std::string endpoint_;
    std::atomic<std::size_t> points_{0};
    std::jthread thread_;
};

TEST(GraspReply, DecodesGraspsInOrder)
{
    const auto head = header(kGraspProtocolVersion, 1);
    const auto body =
        rows({{1, 2, 3, 0, 0, 0, 1, 0.9, 0.05, 0.03}, {4, 5, 6, 0, 0, 1, 0, 0.4, 0.06, 0.02}});
    const auto reply = decodeGraspReply(head, "2 grasps", body);
    ASSERT_TRUE(reply.has_value()) << reply.error();
    ASSERT_EQ(reply->grasps.size(), 2U);
    EXPECT_TRUE(reply->success);
    EXPECT_EQ(reply->message, "2 grasps");
    EXPECT_DOUBLE_EQ(reply->grasps[1].position_m[2], 6.0);
    EXPECT_DOUBLE_EQ(reply->grasps[1].orientation_xyzw[2], 1.0);
    EXPECT_DOUBLE_EQ(reply->grasps[0].score, 0.9);
    EXPECT_DOUBLE_EQ(reply->grasps[0].width_m, 0.05);
    EXPECT_DOUBLE_EQ(reply->grasps[0].object_width_m, 0.03);
}

TEST(GraspReply, KeepsAFailureWithItsMessage)
{
    const auto reply = decodeGraspReply(header(kGraspProtocolVersion, 0), "10 object points", {});
    ASSERT_TRUE(reply.has_value());
    EXPECT_FALSE(reply->success);
    EXPECT_EQ(reply->message, "10 object points");
    EXPECT_TRUE(reply->grasps.empty());
}

TEST(GraspReply, RefusesFramesOffTheProtocol)
{
    EXPECT_FALSE(decodeGraspReply(std::vector<std::byte>(4), "", {}).has_value());
    EXPECT_FALSE(decodeGraspReply(header(kGraspProtocolVersion + 1, 1), "", {}).has_value());
    EXPECT_FALSE(decodeGraspReply(header(kGraspProtocolVersion, 1), "", std::vector<std::byte>(81))
                     .has_value());
}

TEST(GraspGenXClient, SendsThePointsAndReadsTheReply)
{
    const FakeServer server;
    GraspGenXClient client(server.endpoint());
    const std::vector<float> xyz(3 * kPoints, 0.1F);
    const auto reply = client.request(xyz, 0, milliseconds(2000));
    ASSERT_TRUE(reply.has_value()) << reply.error();
    ASSERT_EQ(reply->grasps.size(), 1U);
    EXPECT_DOUBLE_EQ(reply->grasps[0].position_m[1], 0.2);
    EXPECT_EQ(server.points(), kPoints);
}

TEST(GraspGenXClient, RecoversAfterATimeout)
{
    const FakeServer server(1);
    GraspGenXClient client(server.endpoint());
    const std::vector<float> xyz(3 * kPoints, 0.1F);
    const auto late = client.request(xyz, 0, milliseconds(100));
    ASSERT_FALSE(late.has_value());
    EXPECT_NE(late.error().find("no reply"), std::string::npos);
    // The reopened socket sends again; the late reply to the first request is dropped.
    const auto reply = client.request(xyz, 0, milliseconds(2000));
    ASSERT_TRUE(reply.has_value()) << reply.error();
    EXPECT_EQ(reply->grasps.size(), 1U);
}

TEST(GraspGenXClient, RefusesCoordinatesThatAreNotTriples)
{
    GraspGenXClient client("tcp://127.0.0.1:1");
    const std::vector<float> xyz(4, 0.0F);
    EXPECT_FALSE(client.request(xyz, 0, milliseconds(10)).has_value());
}

}  // namespace
}  // namespace piper_perception
