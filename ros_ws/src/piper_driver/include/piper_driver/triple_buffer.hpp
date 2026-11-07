#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace piper_driver
{

/**
 * @class TripleBuffer
 * @brief Hands the newest value from one writer thread to one reader thread without locks.
 * @details write() and read() never block and never allocate. The reader always gets a whole value
 * written by one write(), the newest one completed before its read(), or the one it read last time
 * if nothing newer was written.
 *
 * @tparam T A trivially copyable value type.
 * @warning Exactly one thread may call write() and exactly one thread may call read().
 */
template <typename T>
    requires std::is_trivially_copyable_v<T>
class TripleBuffer
{
public:
    /**
   * @brief Publishes a value to the reader.
   * @param[in] value The value to publish.
   */
    void write(const T & value) noexcept
    {
        slots_[back_] = value;
        // Release: the reader that takes this slot sees the value written above.
        back_ =
            middle_.exchange(static_cast<std::uint8_t>(back_ | kFresh), std::memory_order_acq_rel)
            & kIndex;
    }

    /**
   * @brief Returns the newest published value.
   * @return A reference valid until the next read() on this thread.
   */
    const T & read() noexcept
    {
        if ((middle_.load(std::memory_order_relaxed) & kFresh) != 0)
        {
            front_ = middle_.exchange(front_, std::memory_order_acq_rel) & kIndex;
        }
        return slots_[front_];
    }

private:
    static constexpr std::uint8_t kIndex = 0x3;
    static constexpr std::uint8_t kFresh = 0x4;

    std::array<T, 3> slots_{};
    // The slot between the two sides, with kFresh set while it holds a value the reader has not taken.
    alignas(64) std::atomic<std::uint8_t> middle_{1};
    alignas(64) std::uint8_t back_ = 0;   // Writer's slot.
    alignas(64) std::uint8_t front_ = 2;  // Reader's slot.
};

}  // namespace piper_driver
