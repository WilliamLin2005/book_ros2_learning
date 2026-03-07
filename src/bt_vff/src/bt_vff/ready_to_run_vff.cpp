#include<string>
#include<iostream>
#include<utility>

#include"bt_vff/ready_to_run_vff.hpp"
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"sensor_msgs/msg/laser_scan.hpp"

#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    using namespace std::chrono_literals;

    ready_to_run_vff::ready_to_run_vff(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf)
    :BT::ConditionNode(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        config().blackboard->get("elapsed_time", elapsed_time);
        scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "input_scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::UniquePtr msg_)
            {
                last_scan = std::move(msg_);
                last_scan_time = node_->now();
            });
    }

    BT::NodeStatus ready_to_run_vff::tick()
    {
        if(last_scan==nullptr or node_->now()-last_scan_time>5s)
        {
            RCLCPP_INFO(node_->get_logger(), "No scan received or scan is too old.");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp::Duration elapsed_time_ = node_->now() - last_scan_time;
        double elapsed_time_sec = elapsed_time_.seconds();

        if(elapsed_time_sec >= elapsed_time)
        {
            RCLCPP_INFO(node_->get_logger(), "laser latency is too high");
            return BT::NodeStatus::FAILURE;
        }
        else if(last_scan->ranges[last_scan->ranges.size()/2] < 0.8)
        {
            RCLCPP_INFO(node_->get_logger(), "Obstacle too close");
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }   
    }
} // namespace bt_vff

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_vff::ready_to_run_vff>("ready_to_run_vff");
}