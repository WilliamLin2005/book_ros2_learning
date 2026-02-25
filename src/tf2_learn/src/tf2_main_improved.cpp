#include"tf2_learn/obstacle_detector_improved.hpp"
#include"tf2_learn/obstacle_monitor.hpp"
#include"tf2_learn/odom_meters.hpp"
#include"rclcpp/rclcpp.hpp"
#include<memory>

int main(int argc,char*argv[])
{
    rclcpp::init(argc, argv);
    auto obs_det_imp = std::make_shared<tf2_learn::obs_det_imp_node>("obs_det_imp");
    auto obs_mon=std::make_shared<tf2_learn::obs_mon_node>("obs_mon");
    auto odom_to_robot = std::make_shared<tf2_learn::odom_meters_node>("odom_to_robot");
    rclcpp::executors::SingleThreadedExecutor exec_;
    exec_.add_node(obs_det_imp);
    exec_.add_node(obs_mon);
    exec_.add_node(odom_to_robot);
    exec_.spin();
    rclcpp::shutdown();

    return 0;
}