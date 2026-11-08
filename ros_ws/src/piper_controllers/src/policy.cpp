#include "piper_controllers/policy.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_session_options_config_keys.h>

namespace piper_controllers
{
namespace
{

// One ONNX Runtime environment for the process, made at the first load, outside any control loop.
Ort::Env & environment()
{
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "piper_controllers");
    return env;
}

// A model's batch dimension is 1 or free; its second is the vector's length.
bool fits(const std::vector<int64_t> & shape, std::size_t length)
{
    return shape.size() == 2 && (shape[0] == 1 || shape[0] < 0) && std::cmp_equal(shape[1], length);
}

std::string shapeText(const std::vector<int64_t> & shape)
{
    std::string text;
    for (const int64_t dim : shape)
    {
        text += text.empty() ? std::format("{}", dim) : std::format(" x {}", dim);
    }
    return std::format("[{}]", text);
}

}  // namespace

struct Policy::Impl
{
    Ort::Session session{nullptr};
    Ort::RunOptions run_options;
    std::string input_name;
    std::string output_name;
    // On the heap: the bound tensors point into them, so they must not move with the Policy.
    std::vector<float> observations;
    std::vector<float> actions;
    std::array<int64_t, 2> input_shape{};
    std::array<int64_t, 2> output_shape{};
    Ort::Value input{nullptr};
    Ort::Value output{nullptr};
};

Policy::Policy(std::unique_ptr<Impl> impl) noexcept : impl_(std::move(impl)) {}
Policy::Policy(Policy && other) noexcept = default;
Policy & Policy::operator=(Policy && other) noexcept = default;
Policy::~Policy() = default;

std::expected<Policy, std::string> Policy::load(
    const std::string & path, std::size_t observations, std::size_t actions)
{
    try
    {
        Ort::SessionOptions options;
        // Inference runs inline in the controller's update: one thread, no spinning workers.
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options.AddConfigEntry(kOrtSessionOptionsConfigAllowIntraOpSpinning, "0");

        auto impl = std::make_unique<Impl>();
        impl->session = Ort::Session(environment(), path.c_str(), options);
        if (impl->session.GetInputCount() != 1 || impl->session.GetOutputCount() != 1)
        {
            return std::unexpected(std::format("{} has not one input and one output", path));
        }
        const auto input_shape =
            impl->session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape =
            impl->session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (!fits(input_shape, observations) || !fits(output_shape, actions))
        {
            return std::unexpected(
                std::format(
                    "{} maps {} to {}, the controller needs [1 x {}] to [1 x {}]",
                    path,
                    shapeText(input_shape),
                    shapeText(output_shape),
                    observations,
                    actions));
        }

        const Ort::AllocatorWithDefaultOptions allocator;
        impl->input_name = impl->session.GetInputNameAllocated(0, allocator).get();
        impl->output_name = impl->session.GetOutputNameAllocated(0, allocator).get();
        impl->observations.assign(observations, 0.0F);
        impl->actions.assign(actions, 0.0F);
        impl->input_shape = {1, static_cast<int64_t>(observations)};
        impl->output_shape = {1, static_cast<int64_t>(actions)};
        const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        impl->input = Ort::Value::CreateTensor<float>(
            memory,
            impl->observations.data(),
            impl->observations.size(),
            impl->input_shape.data(),
            impl->input_shape.size());
        impl->output = Ort::Value::CreateTensor<float>(
            memory,
            impl->actions.data(),
            impl->actions.size(),
            impl->output_shape.data(),
            impl->output_shape.size());
        return Policy(std::move(impl));
    }
    catch (const std::exception & error)
    {
        return std::unexpected(std::format("Cannot load the policy {}: {}", path, error.what()));
    }
}

std::span<float> Policy::observations() noexcept
{
    return impl_->observations;
}

std::span<const float> Policy::actions() const noexcept
{
    return impl_->actions;
}

bool Policy::run() noexcept
{
    try
    {
        const char * input = impl_->input_name.c_str();
        const char * output = impl_->output_name.c_str();
        impl_->session.Run(
            impl_->run_options, &input, &impl_->input, 1, &output, &impl_->output, 1);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

}  // namespace piper_controllers
