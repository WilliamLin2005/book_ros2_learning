#include<string>
#include<iostream>
#include<utility>

#include"bt_vff/Turn.hpp"
#include"behaviortree_cpp_v3/behavior_tree.h"

#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    using namespace std::chrono_literals;

    Turn::Turn(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf)
    :BT::ActionNodeBase(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        turn_pub = node_->create_publisher<geometry_msgs::msg::Twist>("vel_output", 10);
        scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "input_scan", rclcpp::SensorDataQoS(), 
            [this]( sensor_msgs::msg::LaserScan::UniquePtr msg_)
            {
                latest_scan = std::move(msg_);
            });
    }

    BT::NodeStatus Turn::tick()
    {
        if(latest_scan==nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }

        if(status()==BT::NodeStatus::IDLE)
        {
            start_time = node_->now();
        }

        geometry_msgs::msg::Twist turn_msg;
        where_to_turn = get_where_to_turn();
        turn_msg.angular.set__z(1.0*(where_to_turn));
        turn_pub->publish(turn_msg);

        auto elapsed = node_->now() - start_time;
        if(elapsed<3s)
        {
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }
    }

    bool Turn::get_where_to_turn()
    {
            auto right_range = latest_scan->ranges[latest_scan->ranges.size()/4];
            auto left_range = latest_scan->ranges[3*latest_scan->ranges.size()/4];
            return left_range>=right_range;
    }
} //namespace bt_vff

#include"behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_vff::Turn>("Turn");
}