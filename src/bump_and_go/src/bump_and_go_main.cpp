#include"rclcpp/rclcpp.hpp"
#include"bump_and_go/bump_and_go.hpp"

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto bump_and_go_node = std::make_shared<bump_and_go::bumpandgonode>("bump_and_go");
    rclcpp::spin(bump_and_go_node);
    rclcpp::shutdown();
    return 0;
}