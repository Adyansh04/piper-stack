#include "piper_skills/camera_capture.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <expected>
#include <format>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace piper_skills
{

CameraCapture::CameraCapture(rclcpp::Node & node, CameraSettings settings, tf2_ros::Buffer & tf)
    : node_(node), settings_(std::move(settings)), tf_(tf)
{
    group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    // Best effort with a small depth matches the driver's and the simulator's publishers alike.
    const rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(2);
    colour_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
        settings_.colour_topic,
        qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            const std::scoped_lock lock(mutex_);
            colour_ = std::move(msg);
        },
        options);
    depth_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
        settings_.depth_topic,
        qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            const std::scoped_lock lock(mutex_);
            depth_ = std::move(msg);
        },
        options);
    info_sub_ = node_.create_subscription<sensor_msgs::msg::CameraInfo>(
        settings_.info_topic,
        qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            const std::scoped_lock lock(mutex_);
            info_ = std::move(msg);
        },
        options);
}

std::expected<CapturedFrame, std::string> CameraCapture::capture(const std::atomic<bool> & cancel)
{
    const rclcpp::Time earliest = node_.now() + rclcpp::Duration(settings_.settle);
    const auto deadline = std::chrono::steady_clock::now() + settings_.settle + settings_.timeout;
    const auto window_ns = std::chrono::nanoseconds(settings_.pair_window).count();
    sensor_msgs::msg::Image::ConstSharedPtr colour;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    while (true)
    {
        if (cancel.load())
        {
            return std::unexpected("cancelled while waiting for the camera");
        }
        {
            const std::scoped_lock lock(mutex_);
            colour = colour_;
            depth = depth_;
            info = info_;
        }
        if (colour && depth && info && rclcpp::Time(colour->header.stamp) >= earliest
            && std::abs((rclcpp::Time(colour->header.stamp) - rclcpp::Time(depth->header.stamp))
                            .nanoseconds())
                   <= window_ns)
        {
            break;
        }
        if (std::chrono::steady_clock::now() > deadline)
        {
            return std::unexpected(
                std::format(
                    "no colour and depth pair within {} ms on {} and {}",
                    (settings_.settle + settings_.timeout).count(),
                    settings_.colour_topic,
                    settings_.depth_topic));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (colour->width != depth->width || colour->height != depth->height)
    {
        return std::unexpected(
            std::format(
                "depth is {}x{}, colour {}x{}: depth must be aligned to colour",
                depth->width,
                depth->height,
                colour->width,
                colour->height));
    }
    CapturedFrame frame;
    frame.colour = colour;
    const cv_bridge::CvImageConstPtr raw = cv_bridge::toCvShare(depth);
    if (depth->encoding == "16UC1" || depth->encoding == "mono16")
    {
        raw->image.convertTo(frame.depth_m, CV_32FC1, 0.001);
    }
    else if (depth->encoding == "32FC1")
    {
        frame.depth_m = raw->image.clone();
    }
    else
    {
        return std::unexpected(
            std::format(
                "depth encoding '{}' is neither 16UC1 millimetres nor 32FC1 metres",
                depth->encoding));
    }
    frame.camera =
        CameraModel{.fx = info->k[0], .fy = info->k[4], .cx = info->k[2], .cy = info->k[5]};
    try
    {
        frame.base_to_camera = tf2::transformToEigen(tf_.lookupTransform(
            settings_.planning_frame,
            colour->header.frame_id,
            colour->header.stamp,
            rclcpp::Duration(settings_.tf_timeout)));
    }
    catch (const tf2::TransformException & error)
    {
        return std::unexpected(
            std::format(
                "no transform {} <- {} at the image's stamp: {}",
                settings_.planning_frame,
                colour->header.frame_id,
                error.what()));
    }
    return frame;
}

}  // namespace piper_skills
