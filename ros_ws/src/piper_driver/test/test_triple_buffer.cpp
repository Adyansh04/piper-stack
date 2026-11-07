#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <thread>

#include "piper_driver/triple_buffer.hpp"

using piper_driver::TripleBuffer;

TEST(TripleBuffer, ReadsTheNewestWrite)
{
    TripleBuffer<int> buffer;
    EXPECT_EQ(buffer.read(), 0);
    buffer.write(1);
    buffer.write(2);
    EXPECT_EQ(buffer.read(), 2);
    EXPECT_EQ(buffer.read(), 2);
    buffer.write(3);
    EXPECT_EQ(buffer.read(), 3);
}

TEST(TripleBuffer, ConcurrentReadsAreWholeAndNeverGoBack)
{
    // Every field of a value holds the same counter, so a torn read shows as unequal fields.
    using Value = std::array<std::uint64_t, 16>;
    TripleBuffer<Value> buffer;
    constexpr std::uint64_t kWrites = 200'000;
    std::jthread writer([&buffer] {
        for (std::uint64_t i = 1; i <= kWrites; ++i)
        {
            Value v;
            v.fill(i);
            buffer.write(v);
        }
    });
    std::uint64_t last = 0;
    while (last < kWrites)
    {
        const Value & v = buffer.read();
        for (std::uint64_t field : v)
        {
            ASSERT_EQ(field, v[0]);
        }
        ASSERT_GE(v[0], last);
        last = v[0];
    }
}
