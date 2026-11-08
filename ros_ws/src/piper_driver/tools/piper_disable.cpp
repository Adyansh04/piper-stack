// Disables all of the arm's motors: a raised arm falls. Only with the arm at rest or held.
//
//   piper_disable <interface> --yes

#include <cstdio>
#include <exception>
#include <print>
#include <string>

#include "piper_driver/arm.hpp"

int main(int argc, char ** argv)
try
{
    if (argc != 3 || std::string(argv[2]) != "--yes")
    {
        std::println(
            stderr, "usage: piper_disable <interface> --yes   (the arm loses torque at once)");
        return 2;
    }
    auto arm = piper_driver::Arm::open(argv[1]);
    if (!arm)
    {
        std::println(stderr, "cannot open {}: {}", argv[1], arm.error().message());
        return 1;
    }
    if (auto sent = (*arm)->enable(false); !sent)
    {
        std::println(stderr, "sending the disable frame failed: {}", sent.error().message());
        return 1;
    }
    std::println("disable sent on {}", argv[1]);
    return 0;
}
catch (const std::exception & e)
{
    std::fputs(e.what(), stderr);
    return 1;
}
