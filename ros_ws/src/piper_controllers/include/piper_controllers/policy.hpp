#pragma once

#include <cstddef>
#include <expected>
#include <memory>
#include <span>
#include <string>

namespace piper_controllers
{

/**
 * @brief A trained policy in ONNX, one observation vector in, one action vector out.
 * @details Inference runs on the CPU in the calling thread. The observation and action buffers
 * are allocated once at load and bound to the model, so a call to run() reads observations() and
 * writes actions() in place.
 *
 * @note ONNX Runtime fills its memory arena during the first runs; later runs reuse it.
 */
class Policy
{
public:
    /**
     * @brief Loads a model and checks that its shapes match.
     * @param[in] path The ONNX file.
     * @param[in] observations Length of the observation vector the caller gives.
     * @param[in] actions Length of the action vector the caller reads.
     * @return The policy, or why it cannot be used: no such file, not one input and one output,
     * or other lengths than given.
     */
    [[nodiscard]] static std::expected<Policy, std::string> load(
        const std::string & path, std::size_t observations, std::size_t actions);

    Policy(Policy && other) noexcept;
    Policy & operator=(Policy && other) noexcept;
    Policy(const Policy &) = delete;
    Policy & operator=(const Policy &) = delete;
    ~Policy();

    /// @brief The observation vector the next run() reads.
    [[nodiscard]] std::span<float> observations() noexcept;

    /// @brief The action vector run() writes.
    [[nodiscard]] std::span<const float> actions() const noexcept;

    /**
     * @brief Runs the model on observations() and writes actions().
     * @return Whether inference succeeded; after a failure actions() is not to be used.
     */
    [[nodiscard]] bool run() noexcept;

private:
    struct Impl;
    explicit Policy(std::unique_ptr<Impl> impl) noexcept;

    std::unique_ptr<Impl> impl_;
};

}  // namespace piper_controllers
