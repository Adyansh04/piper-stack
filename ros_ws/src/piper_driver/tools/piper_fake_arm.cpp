// Runs the fake Piper firmware on a virtual CAN interface until Ctrl-C.
//
//   piper_fake_arm [interface=vcan0]

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <print>
#include <string>
#include <thread>

#include "piper_driver/fake_arm.hpp"

namespace
{
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): set by the signal handler
std::atomic<bool> g_stop{false};
}  // namespace

int main(int argc, char ** argv)
try
{
    piper_driver::FakeArm::Options options;
    if (argc > 1)
    {
        options.interface = argv[1];
    }
    std::signal(SIGINT, [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });
    auto arm = piper_driver::FakeArm::start(options);
    if (!arm)
    {
        std::println(stderr, "cannot open {}: {}", options.interface, arm.error().message());
        return 1;
    }
    std::println("fake Piper on {}; Ctrl-C stops it", options.interface);
    while (!g_stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
    return 0;
}
catch (const std::exception & e)
{
    std::fputs(e.what(), stderr);
    return 1;
}
