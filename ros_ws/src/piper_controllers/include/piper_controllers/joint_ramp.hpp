#pragma once

#include <cstddef>
#include <span>
#include <vector>

namespace piper_controllers
{

/**
 * @brief Joint positions wanted by a policy, approached under velocity and acceleration caps.
 * @details A policy's action, times the scale plus each joint's offset and inside the joint
 * limits, is the joint's wanted position. Every step the wanted velocity is the fastest from
 * which the joint still stops at that position (at most the velocity cap), the commanded
 * velocity moves towards it by at most the acceleration cap times the step, and the position
 * target moves by the commanded velocity, inside the joint limits. The reach task in Isaac Lab
 * trains with this law (`lab/piper_lab/actions.py`), so a policy meets the same motion in
 * training and on the arm.
 */
class JointRamp
{
public:
    /**
     * @param[in] lower_rad Lowest position of each joint, rad.
     * @param[in] upper_rad Highest position of each joint, rad.
     * @param[in] offset_rad Wanted position of each joint at a zero action, rad.
     * @param[in] scale_rad Change of a wanted position per unit of action, rad.
     * @param[in] max_velocity_rad_s Fastest commanded velocity, rad/s.
     * @param[in] max_acceleration_rad_s2 Largest change of the commanded velocity, rad/s^2.
     * @throws std::invalid_argument If the limits and offsets differ in length, a lower limit
     * lies above its upper one, or the scale or a cap is not positive.
     */
    JointRamp(
        std::vector<double> lower_rad,
        std::vector<double> upper_rad,
        std::vector<double> offset_rad,
        double scale_rad,
        double max_velocity_rad_s,
        double max_acceleration_rad_s2);

    /// @brief Number of joints.
    [[nodiscard]] std::size_t size() const noexcept
    {
        return target_rad_.size();
    }

    /**
     * @brief Holds the joints where they are: targets at the measured positions, at rest.
     * @param[in] positions_rad Measured position of each joint, rad.
     */
    void reset(std::span<const double> positions_rad) noexcept;

    /**
     * @brief Sets the wanted positions from a policy's actions, one per joint.
     * @param[in] actions The actions; a non-finite one stops the joints instead (see stop()).
     */
    void want(std::span<const float> actions) noexcept;

    /// @brief Brings every joint to rest where it is, decelerating at the cap.
    void stop() noexcept;

    /**
     * @brief Advances the motion by one control step.
     * @param[in] dt_s Length of the step, s; a step that is not positive moves nothing.
     * @return The position target of each joint, rad, valid until the next call.
     */
    std::span<const double> step(double dt_s) noexcept;

private:
    std::vector<double> lower_rad_;
    std::vector<double> upper_rad_;
    std::vector<double> offset_rad_;
    double scale_rad_;
    double max_velocity_rad_s_;
    double max_acceleration_rad_s2_;
    // Whether the joints move to the wanted positions; otherwise they come to rest.
    bool tracking_ = false;
    std::vector<double> wanted_rad_;
    std::vector<double> velocity_rad_s_;
    std::vector<double> target_rad_;
};

}  // namespace piper_controllers
