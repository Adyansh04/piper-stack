// Prints the arm's decoded feedback ten times a second. Read-only: it never sends a frame.
//
//   piper_monitor [interface=can_follower] [seconds=0: until Ctrl-C]

#include <atomic>
#include <chrono>
#include <csignal>
#include <numbers>
#include <print>
#include <string>
#include <thread>

#include "piper_driver/arm.hpp"

namespace
{
std::atomic<bool> g_stop{false};
}  // namespace

int main(int argc, char ** argv)
{
    const std::string interface = argc > 1 ? argv[1] : "can_follower";
    const double seconds = argc > 2 ? std::stod(argv[2]) : 0.0;
    std::signal(SIGINT, [](int) { g_stop = true; });
    std::signal(SIGTERM, [](int) { g_stop = true; });

    auto arm = piper_driver::Arm::open(interface);
    if (!arm)
    {
        std::println(stderr, "cannot open {}: {}", interface, arm.error().message());
        return 1;
    }
    constexpr double kDeg = 180.0 / std::numbers::pi;
    const auto start = std::chrono::steady_clock::now();
    std::uint64_t last_frames = 0;
    while (!g_stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        const auto & s = (*arm)->latest();
        std::print("joints deg:");
        for (double q : s.position_rad)
        {
            std::print(" {:8.3f}", q * kDeg);
        }
        std::print(" | vel rad/s:");
        for (const auto & m : s.motor)
        {
            std::print(" {:6.3f}", m.velocity_rad_s);
        }
        std::print(" | effort Nm:");
        for (const auto & m : s.motor)
        {
            std::print(" {:6.3f}", m.effort_nm);
        }
        int enabled = 0;
        for (const auto & d : s.driver)
        {
            enabled += (d.status_bits & piper_driver::protocol::kDriverEnabledBit) != 0 ? 1 : 0;
        }
        std::println(
            " | gripper {:.2f} mm | mode {} status {} limits 0x{:02X} comm 0x{:02X} | enabled {}/6 "
            "| "
            "{:.0f} frames/s | foreign {} | bus errors {}",
            s.gripper.stroke_m * 1e3,
            s.status.ctrl_mode,
            s.status.arm_status,
            s.status.joint_beyond_limit,
            s.status.joint_comm_error,
            enabled,
            static_cast<double>(s.frames - last_frames) * 10.0,
            s.foreign_commands,
            s.bus_errors);
        last_frames = s.frames;
        if (seconds > 0.0
            && std::chrono::steady_clock::now() - start > std::chrono::duration<double>(seconds))
        {
            break;
        }
    }
    return 0;
}
