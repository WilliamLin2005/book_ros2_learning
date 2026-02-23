#include"rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char*argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("logger_node");
    rclcpp::Rate looper(500ms);
    int counter = 0;

    while(rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "HEllo %d", counter++);
        rclcpp::spin_some(node);
        looper.sleep();
    }

    rclcpp::shutdown();
    return 0;
}