#pragma once

#include <atomic>
#include <chrono>
#include <expected>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>

#include "piper_skills/object_cloud.hpp"

namespace piper_skills
{

/**
 * @brief One colour image with the depth aligned to it, as the skills use them.
 */
struct CapturedFrame
{
    /**
     * @brief The colour image as the camera sent it (for the segmentation service).
     */
    sensor_msgs::msg::Image::ConstSharedPtr colour;

    /**
     * @brief Depth aligned to the colour image, CV_32FC1, metres; 0 or NaN where there is none.
     */
    cv::Mat depth_m;

    /**
     * @brief The colour camera's intrinsics.
     */
    CameraModel camera;

    /**
     * @brief The colour optical frame in the planning frame at the image's stamp.
     */
    Eigen::Isometry3d base_to_camera{Eigen::Isometry3d::Identity()};
};

/**
 * @brief Topics and timing of the wrist camera.
 */
struct CameraSettings
{
    /**
     * @brief Colour image, depth aligned to it and the colour camera's info.
     */
    std::string colour_topic;
    std::string depth_topic;
    std::string info_topic;

    /**
     * @brief The planning frame the camera's pose is given in.
     */
    std::string planning_frame;

    /**
     * @brief Frames taken earlier than this after a request are skipped: the arm may still swing.
     */
    std::chrono::milliseconds settle{500};

    /**
     * @brief Longest wait for a fresh pair of images.
     */
    std::chrono::milliseconds timeout{5000};

    /**
     * @brief Colour and depth stamps may differ by at most this much to be a pair.
     */
    std::chrono::milliseconds pair_window{50};

    /**
     * @brief Longest wait for the camera's pose in TF.
     */
    std::chrono::milliseconds tf_timeout{500};
};

/**
 * @class CameraCapture
 * @brief Keeps the wrist camera's newest colour, depth and camera info, and hands out pairs.
 * @details The subscriptions run in a callback group of their own, so a capture can wait in another
 * callback while frames keep arriving. Depth comes as 16UC1 millimetres (Percipio's driver) or
 * 32FC1 metres (the simulator); both are returned in metres.
 */
class CameraCapture
{
public:
    /**
     * @brief Subscribes to the camera's topics.
     *
     * @param[in] node The node the subscriptions belong to.
     * @param[in] settings Topics, frame and timing.
     * @param[in] tf A TF buffer filled by a listener elsewhere.
     */
    CameraCapture(rclcpp::Node & node, CameraSettings settings, tf2_ros::Buffer & tf);

    /**
     * @brief Waits for a colour image and its depth taken after `settle`, and the camera's pose.
     *
     * @param[in] cancel Polled while waiting; set, the wait ends early.
     *
     * @return The frame, or why there is none: a timeout, a cancel, mismatched sizes, an unknown depth
     *         encoding or a missing transform.
     */
    [[nodiscard]] std::expected<CapturedFrame, std::string> capture(
        const std::atomic<bool> & cancel);

private:
    rclcpp::Node & node_;
    CameraSettings settings_;
    tf2_ros::Buffer & tf_;
    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colour_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    // Guards the three latest messages, written by the subscriptions and read by capture().
    std::mutex mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr colour_;
    sensor_msgs::msg::Image::ConstSharedPtr depth_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
};

}  // namespace piper_skills
