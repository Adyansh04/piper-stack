#include "piper_skills/task_stages.hpp"

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <format>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace piper_skills
{
namespace
{

constexpr double kDegToRad = std::numbers::pi / 180.0;

YAML::Node require(const YAML::Node & node, const std::string & key, const std::string & where)
{
    if (!node[key])
    {
        throw std::invalid_argument(std::format("{} has no {}", where, key));
    }
    return node[key];
}

double positive(const YAML::Node & node, const std::string & key, const std::string & where)
{
    const auto value = require(node, key, where).as<double>();
    if (!(value > 0.0))
    {
        throw std::invalid_argument(
            std::format("{}.{} must be positive, not {}", where, key, value));
    }
    return value;
}

// xyz in metres and roll, pitch, yaw in degrees about the fixed axes, as task.yaml and URDF.
Eigen::Isometry3d pose(const YAML::Node & node, const std::string & where)
{
    const auto vector = [&](const char * key) {
        if (!node[key])
        {
            return Eigen::Vector3d::Zero().eval();
        }
        const auto values = node[key].as<std::vector<double>>();
        if (values.size() != 3)
        {
            throw std::invalid_argument(std::format("{}.{} has 3 values", where, key));
        }
        return Eigen::Vector3d(values[0], values[1], values[2]);
    };
    const Eigen::Vector3d rpy = vector("rpy") * kDegToRad;
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = vector("xyz");
    out.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                       .toRotationMatrix();
    return out;
}

StageMotion motionOf(const std::string & name, const std::string & where)
{
    if (name == "insert")
    {
        return StageMotion::kInsert;
    }
    if (name == "screw")
    {
        return StageMotion::kScrew;
    }
    if (name == "twist")
    {
        return StageMotion::kTwist;
    }
    throw std::invalid_argument(
        std::format("{}.motion is insert, screw or twist, not {}", where, name));
}

Stage parseStage(const YAML::Node & node, const YAML::Node & scene, const std::string & where)
{
    Stage stage;
    stage.name = require(node, "name", where).as<std::string>();
    stage.part = require(node, "part", where).as<std::string>();
    if (scene["start"] && scene["start"][stage.part] && scene["start"][stage.part]["in"])
    {
        stage.support = scene["start"][stage.part]["in"].as<std::string>();
    }
    const auto goal = scene["goal"] ? scene["goal"][stage.part] : YAML::Node();
    if (!goal || !goal["in"])
    {
        throw std::invalid_argument(
            std::format("{}: scene.goal places {} in no other part's frame", where, stage.part));
    }
    stage.mate = goal["in"].as<std::string>();
    stage.goal = pose(goal, "scene.goal." + stage.part);
    const auto grasp = require(node, "grasp", where);
    stage.grasp = pose(grasp, where + ".grasp");
    stage.opening_m = positive(grasp, "opening_m", where + ".grasp");
    stage.symmetry = grasp["symmetry"] ? grasp["symmetry"].as<int>() : 1;
    if (stage.symmetry != 1 && stage.symmetry != 2)
    {
        throw std::invalid_argument(std::format("{}.grasp.symmetry is 1 or 2", where));
    }
    stage.approach_m = positive(node, "approach_m", where);
    stage.release_m = require(node, "release_m", where).as<double>();
    if (node["approach_axis"])
    {
        const auto axis = node["approach_axis"].as<std::vector<double>>();
        if (axis.size() != 3 || Eigen::Vector3d(axis[0], axis[1], axis[2]).norm() < 1e-9)
        {
            throw std::invalid_argument(
                std::format("{}.approach_axis is a 3-vector, not 0", where));
        }
        stage.approach_axis = Eigen::Vector3d(axis[0], axis[1], axis[2]).normalized();
    }
    if (stage.release_m < 0.0)
    {
        throw std::invalid_argument(std::format("{}.release_m must not be negative", where));
    }
    stage.motion = motionOf(require(node, "motion", where).as<std::string>(), where);
    if (stage.motion == StageMotion::kScrew)
    {
        const auto screw = require(node, "screw", where);
        stage.pitch_m = positive(screw, "pitch_m", where + ".screw");
        stage.turns = positive(screw, "turns", where + ".screw");
    }
    if (stage.motion == StageMotion::kTwist)
    {
        stage.twist_rad = require(node, "twist_deg", where).as<double>() * kDegToRad;
        if (stage.twist_rad == 0.0)
        {
            throw std::invalid_argument(std::format("{}.twist_deg must not be 0", where));
        }
    }
    if (const auto tolerance = node["goal_tolerance"])
    {
        stage.goal_tolerance_m = positive(tolerance, "position_m", where + ".goal_tolerance");
        stage.goal_tolerance_rad =
            positive(tolerance, "angle_deg", where + ".goal_tolerance") * kDegToRad;
    }
    for (const auto & pair : node["contacts"])
    {
        const auto names = pair.as<std::vector<std::string>>();
        if (names.size() != 2)
        {
            throw std::invalid_argument(std::format("{}.contacts holds pairs of names", where));
        }
        stage.contacts.emplace_back(names[0], names[1]);
    }
    return stage;
}

}  // namespace

std::vector<Stage> parseStages(std::string_view task_yaml)
{
    YAML::Node spec;
    try
    {
        spec = YAML::Load(std::string(task_yaml));
    }
    catch (const YAML::Exception & error)
    {
        throw std::invalid_argument(std::format("task.yaml does not parse: {}", error.what()));
    }
    if (!spec["stages"] || !spec["stages"].IsSequence() || spec["stages"].size() == 0)
    {
        throw std::invalid_argument("task.yaml has no stages");
    }
    const auto scene = spec["scene"] ? spec["scene"] : YAML::Node();
    std::vector<Stage> stages;
    std::set<std::string> names;
    try
    {
        for (std::size_t i = 0; i < spec["stages"].size(); ++i)
        {
            stages.push_back(parseStage(spec["stages"][i], scene, std::format("stages[{}]", i)));
            if (!names.insert(stages.back().name).second)
            {
                throw std::invalid_argument(
                    std::format("two stages are called {}", stages.back().name));
            }
        }
    }
    catch (const YAML::Exception & error)
    {
        throw std::invalid_argument(std::format("task.yaml's stages: {}", error.what()));
    }
    return stages;
}

double stageTurn(const Stage & stage)
{
    switch (stage.motion)
    {
        case StageMotion::kScrew:
            return -2.0 * std::numbers::pi * stage.turns;
        case StageMotion::kTwist:
            return stage.twist_rad;
        case StageMotion::kInsert:
            break;
    }
    return 0.0;
}

Eigen::Isometry3d partOnMate(const Stage & stage, double left)
{
    const double lift_m = stage.motion == StageMotion::kScrew ? stage.pitch_m * stage.turns : 0.0;
    Eigen::Isometry3d out = stage.goal;
    out.translate(
        (stage.release_m * stage.approach_axis) + Eigen::Vector3d(0.0, 0.0, left * lift_m));
    out.rotate(Eigen::AngleAxisd(-left * stageTurn(stage), Eigen::Vector3d::UnitZ()));
    return out;
}

std::vector<Eigen::Isometry3d> stageGrasps(const Stage & stage)
{
    std::vector<Eigen::Isometry3d> out;
    for (int k = 0; k < stage.symmetry; ++k)
    {
        const double turn = 2.0 * std::numbers::pi * k / stage.symmetry;
        out.emplace_back(Eigen::AngleAxisd(turn, Eigen::Vector3d::UnitZ()) * stage.grasp);
    }
    return out;
}

}  // namespace piper_skills
