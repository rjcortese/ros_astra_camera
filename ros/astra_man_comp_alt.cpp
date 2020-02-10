#include <memory>
#include <string>
#include <vector>

// might be able to split this into three components
// one for each camera (ir, color, depth)
#include "astra_camera/astra_driver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // might need a separate thread for each camera...
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    std::vector<std::string> args(argv + 1, argv + argc);
    options.arguments(args);

    auto astra_comp = std::make_shared<astra_wrapper::AstraDriver>(options);
    exec.add_node(astra_comp);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
