#include "piper_estimation/wrench_estimator.hpp"

#include <Eigen/Core>
#include <Eigen/QR>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace piper_estimation
{

namespace
{
constexpr double kGravityMps2 = 9.81;

KDL::JntArray toKdl(const Eigen::VectorXd & v)
{
    KDL::JntArray out(static_cast<unsigned int>(v.size()));
    out.data = v;
    return out;
}
}  // namespace

std::optional<WrenchEstimator> WrenchEstimator::fromUrdf(
    const std::string & urdf, const std::string & base_link, const std::string & tip_link)
{
    KDL::Tree tree;
    KDL::Chain chain;
    if (!kdl_parser::treeFromString(urdf, tree) || !tree.getChain(base_link, tip_link, chain))
    {
        return std::nullopt;
    }
    return WrenchEstimator(chain);
}

WrenchEstimator::WrenchEstimator(const KDL::Chain & chain)
    : chain_(std::make_unique<KDL::Chain>(chain))
    , dynamics_(std::make_unique<KDL::ChainDynParam>(*chain_, KDL::Vector(0.0, 0.0, -kGravityMps2)))
    , jacobian_(std::make_unique<KDL::ChainJntToJacSolver>(*chain_))
{}

unsigned int WrenchEstimator::joints() const noexcept
{
    return chain_->getNrOfJoints();
}

Eigen::VectorXd WrenchEstimator::gravityTorque(const Eigen::VectorXd & q)
{
    KDL::JntArray gravity(joints());
    dynamics_->JntToGravity(toKdl(q), gravity);
    return gravity.data;
}

Eigen::Matrix<double, 6, 1> WrenchEstimator::wrench(
    const Eigen::VectorXd & q, const Eigen::VectorXd & tau)
{
    KDL::Jacobian jac(joints());
    jacobian_->JntToJac(toKdl(q), jac);
    const Eigen::VectorXd external = tau - gravityTorque(q);
    // tau_ext = J^T F, solved in the least-squares sense (exact for a non-singular 6-joint arm).
    return jac.data.transpose().completeOrthogonalDecomposition().solve(external);
}

GraspDetector::GraspDetector(double effort_threshold_nm, double min_opening_m, int samples) noexcept
    : effort_threshold_nm_(effort_threshold_nm), min_opening_m_(min_opening_m), samples_(samples)
{}

bool GraspDetector::update(double effort_nm, double opening_m) noexcept
{
    const bool pressing = std::abs(effort_nm) > effort_threshold_nm_ && opening_m > min_opening_m_;
    count_ = pressing ? count_ + 1 : 0;
    return count_ >= samples_;
}

}  // namespace piper_estimation
