#ifndef BT_BUMPGO__READY_TO_RUN_VFF_HPP_
#define BT_BUMPGO__READY_TO_RUN_VFF_HPP_

#include<string>

//behavior tree
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"behaviortree_cpp_v3/bt_factory.h"

//ros2
#include"sensor_msgs/msg/laser_scan.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
{
    class ready_to_run_vff:public BT::ConditionNode
    {
        public:
            explicit ready_to_run_vff(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf);

            BT::NodeStatus tick();

            static BT::PortsList providedPorts()
            {
                return BT::PortsList({BT::InputPort<double>("elapsed_time")});
            }

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
            rclcpp::Time last_scan_time;
            sensor_msgs::msg::LaserScan::UniquePtr last_scan;
            double elapsed_time;
    };
}


#endif