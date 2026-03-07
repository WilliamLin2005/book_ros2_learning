#include<string>
#include<iostream>

#include"bt_vff/Stop.hpp"
#include"behaviortree_cpp_v3/behavior_tree.h"

#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    using namespace std::chrono_literals;

    Stop::Stop(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf)
    :BT::ActionNodeBase(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        stop_pub = node_->create_publisher<geometry_msgs::msg::Twist>("vel_output", 10);
    }

    BT::NodeStatus Stop::tick()
    {
        geometry_msgs::msg::Twist stop_msg;
        stop_pub->publish(stop_msg);
        return BT::NodeStatus::SUCCESS;
    }
} //namespace bt_vff

#include"behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_vff::Stop>("Stop");
}