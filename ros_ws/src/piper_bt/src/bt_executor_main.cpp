#include <cstddef>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "piper_bt/bt_executor.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<piper_bt::BtExecutor> node;
    try
    {
        node = piper_bt::BtExecutor::create();
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("bt_executor"), "Cannot start: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    // The tree ticks on its own thread and TF listens on its own; these serve the run_tree server
    // and the leaves' client callbacks.
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), static_cast<std::size_t>(node->executorThreads()));
    executor.add_node(node);
    executor.spin();
    node->finish();
    rclcpp::shutdown();
    return 0;
}
