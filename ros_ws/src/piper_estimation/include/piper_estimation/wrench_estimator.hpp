#pragma once

#include <Eigen/Core>
#include <memory>
#include <optional>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace piper_estimation
{

/**
 * @brief Estimates the external wrench at the tool from the joint torques the motors report.
 * @details The torque left after the gravity model of the arm (from the URDF's masses) is taken
 * as external, and mapped to a wrench at the tip through the pseudo-inverse of the Jacobian's
 * transpose. Friction and the motors' current-to-torque error stay in the estimate: it suits
 * contact detection and coarse force feedback, not force control.
 */
class WrenchEstimator
{
public:
    /**
     * @brief Builds the model of the chain from a URDF.
     * @param[in] urdf The robot description.
     * @param[in] base_link First link of the chain; gravity and the wrench are expressed in it.
     * @param[in] tip_link Link at which the wrench is estimated.
     * @return The estimator, or nothing if the URDF does not parse or holds no such chain.
     */
    [[nodiscard]] static std::optional<WrenchEstimator> fromUrdf(
        const std::string & urdf, const std::string & base_link, const std::string & tip_link);

    /// @brief Number of movable joints in the chain.
    [[nodiscard]] unsigned int joints() const noexcept;

    /**
     * @brief Gravity torque per joint at a pose.
     * @param[in] q Joint positions, rad.
     * @return Torque per joint, N m.
     */
    [[nodiscard]] Eigen::VectorXd gravityTorque(const Eigen::VectorXd & q);

    /**
     * @brief Estimated external wrench at the tip, in the base frame.
     * @param[in] q Joint positions, rad.
     * @param[in] tau Measured joint torques, N m.
     * @return Force (N) then torque (N m), six values.
     */
    [[nodiscard]] Eigen::Matrix<double, 6, 1> wrench(
        const Eigen::VectorXd & q, const Eigen::VectorXd & tau);

private:
    explicit WrenchEstimator(const KDL::Chain & chain);

    // On the heap: KDL's solvers keep a reference to it, which must survive moves of this object.
    std::unique_ptr<KDL::Chain> chain_;
    std::unique_ptr<KDL::ChainDynParam> dynamics_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_;
};

/**
 * @brief Decides whether the gripper holds an object from its torque and opening.
 * @details Held: the grip torque stays above a threshold for a number of consecutive samples
 * while the fingers are still apart (an empty gripper closes fully).
 */
class GraspDetector
{
public:
    /**
     * @param[in] effort_threshold_nm Grip torque above which the fingers press on something.
     * @param[in] min_opening_m Opening below which the gripper counts as closed on nothing.
     * @param[in] samples Consecutive samples the condition must hold.
     */
    GraspDetector(double effort_threshold_nm, double min_opening_m, int samples) noexcept;

    /**
     * @brief Feeds one gripper sample.
     * @param[in] effort_nm Grip torque, N m.
     * @param[in] opening_m Opening between the fingers, m.
     * @return Whether an object is held.
     */
    bool update(double effort_nm, double opening_m) noexcept;

private:
    double effort_threshold_nm_;
    double min_opening_m_;
    int samples_;
    int count_ = 0;
};

}  // namespace piper_estimation
