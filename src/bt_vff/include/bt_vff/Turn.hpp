#ifndef BT_VFF__TURN_HPP_
#define BT_VFF__TURN_HPP_

#include<string>

//behavior tree
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"behaviortree_cpp_v3/bt_factory.h"

//ros2
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    class Turn:public BT::ActionNodeBase
    {
        public:
            explicit Turn(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf);
            void halt() {};
            BT::NodeStatus tick();
            bool get_where_to_turn();
            static BT::PortsList providedPorts()
            {
                return BT::PortsList({});
            }
        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turn_pub;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
            sensor_msgs::msg::LaserScan::UniquePtr latest_scan;
            bool where_to_turn;
            rclcpp::Time start_time;
    };
}

#endif