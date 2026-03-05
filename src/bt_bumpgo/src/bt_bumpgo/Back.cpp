#include<iostream>
#include<string>

#include"behaviortree_cpp_v3/behavior_tree.h"
#include"bt_bumpgo/Back.hpp"

#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{
    using namespace std::chrono_literals;

    Back::Back(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf)
    :BT::ActionNodeBase(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        back_pub = node_->create_publisher<geometry_msgs::msg::Twist>("vel_output", 10);
    }

    BT::NodeStatus Back::tick()
    {
        if(status()==BT::NodeStatus::IDLE)
        {
            start_time_ = node_->now();
        }

        geometry_msgs::msg::Twist back_msg;
        back_msg.linear.set__x(-1.0);
        back_pub->publish(back_msg);

        auto elapsed = node_->now() - start_time_;
        if(elapsed<3s)
        {
            return BT::NodeStatus::RUNNING;
        }
        else{
            return BT::NodeStatus::SUCCESS;
        }
    }
} //namespace bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_bumpgo::Back>("Back");
}