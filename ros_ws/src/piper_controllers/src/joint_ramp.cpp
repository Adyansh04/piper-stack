#include "piper_controllers/joint_ramp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace piper_controllers
{

JointRamp::JointRamp(
    std::vector<double> lower_rad,
    std::vector<double> upper_rad,
    std::vector<double> offset_rad,
    double scale_rad,
    double max_velocity_rad_s,
    double max_acceleration_rad_s2)
    : lower_rad_(std::move(lower_rad))
    , upper_rad_(std::move(upper_rad))
    , offset_rad_(std::move(offset_rad))
    , scale_rad_(scale_rad)
    , max_velocity_rad_s_(max_velocity_rad_s)
    , max_acceleration_rad_s2_(max_acceleration_rad_s2)
    , wanted_rad_(lower_rad_.size(), 0.0)
    , velocity_rad_s_(lower_rad_.size(), 0.0)
    , target_rad_(lower_rad_.size(), 0.0)
{
    if (lower_rad_.size() != upper_rad_.size() || lower_rad_.size() != offset_rad_.size())
    {
        throw std::invalid_argument("JointRamp: the limits and offsets differ in length");
    }
    for (std::size_t i = 0; i < lower_rad_.size(); ++i)
    {
        if (!(lower_rad_[i] <= upper_rad_[i]))
        {
            throw std::invalid_argument("JointRamp: a lower limit lies above its upper limit");
        }
    }
    if (!(scale_rad_ > 0.0) || !(max_velocity_rad_s_ > 0.0) || !(max_acceleration_rad_s2_ > 0.0))
    {
        throw std::invalid_argument("JointRamp: the scale and the caps must be positive");
    }
}

void JointRamp::reset(std::span<const double> positions_rad) noexcept
{
    for (std::size_t i = 0; i < target_rad_.size() && i < positions_rad.size(); ++i)
    {
        target_rad_[i] = std::clamp(positions_rad[i], lower_rad_[i], upper_rad_[i]);
    }
    std::ranges::fill(velocity_rad_s_, 0.0);
    tracking_ = false;
}

void JointRamp::want(std::span<const float> actions) noexcept
{
    if (actions.size() < wanted_rad_.size()
        || !std::ranges::all_of(actions, [](float action) { return std::isfinite(action); }))
    {
        stop();
        return;
    }
    for (std::size_t i = 0; i < wanted_rad_.size(); ++i)
    {
        wanted_rad_[i] = std::clamp(
            offset_rad_[i] + scale_rad_ * static_cast<double>(actions[i]),
            lower_rad_[i],
            upper_rad_[i]);
    }
    tracking_ = true;
}

void JointRamp::stop() noexcept
{
    tracking_ = false;
}

std::span<const double> JointRamp::step(double dt_s) noexcept
{
    if (!(dt_s > 0.0))
    {
        return target_rad_;
    }
    const double change_rad_s = max_acceleration_rad_s2_ * dt_s;
    for (std::size_t i = 0; i < target_rad_.size(); ++i)
    {
        double wanted_rad_s = 0.0;
        if (tracking_)
        {
            // The fastest speed that still stops at the wanted position, and no overshoot in a step.
            const double error_rad = wanted_rad_[i] - target_rad_[i];
            const double distance_rad = std::abs(error_rad);
            const double speed_rad_s = std::min(
                {max_velocity_rad_s_,
                 std::sqrt(2.0 * max_acceleration_rad_s2_ * distance_rad),
                 distance_rad / dt_s});
            wanted_rad_s = std::copysign(speed_rad_s, error_rad);
        }
        velocity_rad_s_[i] +=
            std::clamp(wanted_rad_s - velocity_rad_s_[i], -change_rad_s, change_rad_s);
        target_rad_[i] =
            std::clamp(target_rad_[i] + velocity_rad_s_[i] * dt_s, lower_rad_[i], upper_rad_[i]);
    }
    return target_rad_;
}

}  // namespace piper_controllers
