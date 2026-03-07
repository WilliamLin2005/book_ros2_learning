#ifndef BT_VFF__TURN_HPP_
#define BT_VFF__TURN_HPP_

#include<string>

//behavior tree
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"behaviortree_cpp_v3/bt_factory.h"

//ros2
#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    class Stop:public BT::ActionNodeBase
    {
        public:
            explicit Stop(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf);
            void halt() {};
            BT::NodeStatus tick();

            static BT::PortsList providedPorts()
            {
                return BT::PortsList({});
            }
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub;
    };
}

#endif