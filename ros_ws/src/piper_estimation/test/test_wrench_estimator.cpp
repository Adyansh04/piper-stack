#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <string>

#include "piper_estimation/wrench_estimator.hpp"

namespace
{

using piper_estimation::GraspDetector;
using piper_estimation::WrenchEstimator;

// Two 1 kg links of 0.5 m, revolute about y, hanging off a fixed base: a planar arm whose gravity
// torques are known in closed form.
const std::string kUrdf = R"(<robot name="arm">
  <link name="base"/>
  <link name="l1"><inertial><origin xyz="0.25 0 0"/><mass value="1"/>
    <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/></inertial></link>
  <link name="l2"><inertial><origin xyz="0.25 0 0"/><mass value="1"/>
    <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/></inertial></link>
  <link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="0.5 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="t" type="fixed"><parent link="l2"/><child link="tip"/><origin xyz="0.5 0 0"/></joint>
</robot>)";

TEST(WrenchEstimator, GravityOfAHorizontalArm)
{
    auto estimator = WrenchEstimator::fromUrdf(kUrdf, "base", "tip");
    ASSERT_TRUE(estimator.has_value());
    // NOLINTBEGIN(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when it is empty
    EXPECT_EQ(estimator->joints(), 2U);
    const Eigen::VectorXd g = estimator->gravityTorque(Eigen::Vector2d::Zero());
    // Stretched out along x: 9.81 * (1 * 0.25 + 1 * 0.75) at j1, 9.81 * 0.25 at j2, in N m; the
    // sign is the torque about +y that holds the arm against gravity.
    EXPECT_NEAR(std::abs(g(0)), 9.81, 1e-6);
    EXPECT_NEAR(std::abs(g(1)), 9.81 * 0.25, 1e-6);
    // NOLINTEND(bugprone-unchecked-optional-access)
}

TEST(WrenchEstimator, NoContactGivesNoWrenchAndContactIsRecovered)
{
    auto estimator = WrenchEstimator::fromUrdf(kUrdf, "base", "tip");
    ASSERT_TRUE(estimator.has_value());
    // NOLINTBEGIN(bugprone-unchecked-optional-access): ASSERT_TRUE above returns when it is empty
    const Eigen::Vector2d q(0.3, -0.7);
    const Eigen::VectorXd gravity = estimator->gravityTorque(q);
    EXPECT_LT(estimator->wrench(q, gravity).norm(), 1e-9);
    // A 5 N push straight down at the tip of the stretched-out arm: 5 N m at j2 (0.5 m), 10 N m at
    // j1 (1 m). The planar arm can only see the force along z and the torque about y.
    const Eigen::Vector2d q0(0.0, 0.0);
    const Eigen::VectorXd tau =
        estimator->gravityTorque(q0) + Eigen::Vector2d(5.0 * 1.0, 5.0 * 0.5);
    const auto w = estimator->wrench(q0, tau);
    EXPECT_NEAR(std::abs(w(2)), 5.0, 1e-6);  // N
    // NOLINTEND(bugprone-unchecked-optional-access)
}

TEST(WrenchEstimator, RejectsAMissingChain)
{
    EXPECT_FALSE(WrenchEstimator::fromUrdf(kUrdf, "base", "nowhere").has_value());
    EXPECT_FALSE(WrenchEstimator::fromUrdf("<robot", "base", "tip").has_value());
}

TEST(GraspDetector, NeedsTorqueOpeningAndPersistence)
{
    GraspDetector grasp(0.3, 0.004, 3);
    EXPECT_FALSE(grasp.update(0.5, 0.02));
    EXPECT_FALSE(grasp.update(0.5, 0.02));
    EXPECT_TRUE(grasp.update(0.5, 0.02));
    EXPECT_FALSE(grasp.update(0.5, 0.001));  // closed on nothing
    EXPECT_FALSE(grasp.update(0.1, 0.02));   // open, no torque
}

}  // namespace
