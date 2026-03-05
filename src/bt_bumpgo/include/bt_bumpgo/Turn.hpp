#ifndef BT_BUMPGO__TURN_HPP_
#define BT_BUMPGO__TURN_HPP_

#include<string>

//behavior tree
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"behaviortree_cpp_v3/bt_factory.h"

//ros2
#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{
    class Turn:public BT::ActionNodeBase
    {
        public:
            explicit Turn(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf);
            void halt() {};
            BT::NodeStatus tick();

            static BT::PortsList Portsprovided()
            {
                return BT::PortsList({});
            }
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turn_pub;
            rclcpp::Time start_time;
    };
}

#endif