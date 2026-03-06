#include<string>
#include<utility>

#include"bt_bumpgo/IsObstacle.hpp"
#include"behaviortree_cpp_v3/behavior_tree.h"

#include"sensor_msgs/msg/laser_scan.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_bumpgo
{
    using namespace std::chrono_literals;

    IsObstacle::IsObstacle(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf):
    BT::ConditionNode(xml_pkg_name,conf)
    {
        config().blackboard->get("node_name", node_);
        laser_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::UniquePtr msg_)
                                                                            { this->laser_callback(std::move(msg_)); });
        last_scan_time = node_->now();
    }

    void IsObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg_)
    {
        last_scan = std::move(msg_);
        last_scan_time= node_->now();
    }

    BT::NodeStatus IsObstacle::tick()
    {
        if(last_scan==nullptr)
        {
            RCLCPP_INFO(node_->get_logger(), "laser reading not catch");
            return BT::NodeStatus::FAILURE;
        }
        else if(node_->now()-last_scan_time>1s)
        {
            RCLCPP_INFO(node_->get_logger(), "laser latency is too high");
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            double obs_dist = 1.00;
            getInput("obs_distance", obs_dist);
            auto laser_detect_dist = last_scan->ranges[last_scan->ranges.size() / 2];
            if (laser_detect_dist< obs_dist)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "obstacle detected at:" << laser_detect_dist);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
    }
} //namespace bt_bumpgo

#include"behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_bumpgo::IsObstacle>("IsObstacle");
}