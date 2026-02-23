#include"tf2_learn/obstacle_detector.hpp"
#include"tf2_learn/obstacle_monitor.hpp"
#include"rclcpp/rclcpp.hpp"
#include<memory>

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto obs_det_ = std::make_shared<tf2_learn::obs_det_node>("obs_det_");
    auto obs_mon_=std::make_shared<tf2_learn::obs_mon_node>("obs_mon_");
    rclcpp::executors::SingleThreadedExecutor exec_;
    exec_.add_node(obs_det_);
    exec_.add_node(obs_mon_);
    exec_.spin();
    rclcpp::shutdown();

    return 0;
}