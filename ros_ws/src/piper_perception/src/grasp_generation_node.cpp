#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <piper_msgs/msg/grasp.hpp>
#include <piper_msgs/srv/compute_grasps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "piper_perception/graspgenx_client.hpp"

namespace piper_perception
{

namespace
{

/**
 * @brief Copies a cloud's x, y, z into one contiguous array, whatever its point layout.
 *
 * @throws std::runtime_error If the cloud has no float32 x, y or z field.
 */
std::vector<float> xyzOf(const sensor_msgs::msg::PointCloud2 & cloud)
{
    std::vector<float> xyz;
    xyz.reserve(static_cast<std::size_t>(cloud.width) * cloud.height * 3);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
        xyz.insert(xyz.end(), {*x, *y, *z});
    }
    return xyz;
}

}  // namespace

/**
 * @class GraspGenerationNode
 * @brief Serves `compute_grasps` (piper_msgs/srv/ComputeGrasps) by forwarding each request to the
 *        GraspGenX server.
 * @details
 * The server runs in its own Python environment (graspgenx/graspgenx_server.py); this node puts
 * it on the ROS graph. A request blocks its callback until the reply or `timeout_s`, so the node
 * runs in a multithreaded executor of its own or in `component_container_mt`.
 */
class GraspGenerationNode : public rclcpp::Node
{
public:
    /**
     * @brief Reads the parameters, connects to the server and offers the service.
     *
     * @param[in] options Node options from the component container.
     *
     * @throws std::invalid_argument If `timeout_s` is not positive.
     * @throws zmq::error_t If `server_endpoint` is malformed.
     */
    explicit GraspGenerationNode(const rclcpp::NodeOptions & options)
        : rclcpp::Node("grasp_generation", options)
        , client_(declare_parameter<std::string>("server_endpoint", "tcp://127.0.0.1:5561"))
        , timeout_(timeoutFromParameter(declare_parameter<double>("timeout_s", 60.0)))
    {
        service_ = create_service<piper_msgs::srv::ComputeGrasps>(
            "compute_grasps",
            [this](
                const std::shared_ptr<piper_msgs::srv::ComputeGrasps::Request> & request,
                const std::shared_ptr<piper_msgs::srv::ComputeGrasps::Response> & response) {
                onRequest(*request, *response);
            });
        RCLCPP_INFO(
            get_logger(),
            "compute_grasps forwards to the GraspGenX server at %s",
            get_parameter("server_endpoint").as_string().c_str());
    }

private:
    static std::chrono::milliseconds timeoutFromParameter(double timeout_s)
    {
        if (!(timeout_s > 0.0))
        {
            throw std::invalid_argument(std::format("timeout_s {} is not positive", timeout_s));
        }
        return std::chrono::milliseconds(static_cast<std::int64_t>(timeout_s * 1000.0));
    }

    void onRequest(
        const piper_msgs::srv::ComputeGrasps::Request & request,
        piper_msgs::srv::ComputeGrasps::Response & response)
    {
        std::vector<float> xyz;
        try
        {
            xyz = xyzOf(request.object_cloud);
        }
        catch (const std::exception & error)
        {
            response.success = false;
            response.message = std::format("unreadable object_cloud: {}", error.what());
            return;
        }
        const auto reply = client_.request(xyz, request.num_grasps, timeout_);
        if (!reply)
        {
            response.success = false;
            response.message = reply.error();
            RCLCPP_ERROR(get_logger(), "compute_grasps: %s", reply.error().c_str());
            return;
        }
        response.success = reply->success;
        response.message = reply->message;
        response.grasps.reserve(reply->grasps.size());
        for (const GraspCandidate & candidate : reply->grasps)
        {
            piper_msgs::msg::Grasp & grasp = response.grasps.emplace_back();
            grasp.pose.header = request.object_cloud.header;
            grasp.pose.pose.position.x = candidate.position_m[0];
            grasp.pose.pose.position.y = candidate.position_m[1];
            grasp.pose.pose.position.z = candidate.position_m[2];
            grasp.pose.pose.orientation.x = candidate.orientation_xyzw[0];
            grasp.pose.pose.orientation.y = candidate.orientation_xyzw[1];
            grasp.pose.pose.orientation.z = candidate.orientation_xyzw[2];
            grasp.pose.pose.orientation.w = candidate.orientation_xyzw[3];
            grasp.width_m = static_cast<float>(candidate.width_m);
            grasp.object_width_m = static_cast<float>(candidate.object_width_m);
            grasp.score = static_cast<float>(candidate.score);
        }
    }

    // Used only from the service callback, which the node's default group serialises.
    GraspGenXClient client_;
    std::chrono::milliseconds timeout_;
    rclcpp::Service<piper_msgs::srv::ComputeGrasps>::SharedPtr service_;
};

}  // namespace piper_perception

RCLCPP_COMPONENTS_REGISTER_NODE(piper_perception::GraspGenerationNode)
