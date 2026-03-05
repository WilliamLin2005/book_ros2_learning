#include<string>
#include<iostream>

#include"bt_bumpgo/Forward.hpp"
#include"behaviortree_cpp_v3/behavior_tree.h"

#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{
    Forward::Forward(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf):
    BT::ActionNodeBase(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        twist_pub = node_->create_publisher<geometry_msgs::msg::Twist>("vel_output", 10);
    }

    BT::NodeStatus Forward::tick()
    {
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.set__x(1.0);
        twist_pub->publish(twist_msg);

        return BT::NodeStatus::RUNNING;
    }
} //namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_bumpgo::Forward>("Forward");
}