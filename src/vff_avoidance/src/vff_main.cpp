#include"rclcpp/rclcpp.hpp"
#include"vff_avoidance/vff_avoidance.hpp"
#include<memory>

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto vff_node = std::make_shared<vff_avoidance::vff_avoidance_node>("vff_avoidance");
    rclcpp::spin(vff_node);
    rclcpp::shutdown();
    return 0;
}