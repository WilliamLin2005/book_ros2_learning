#ifndef TF2_LEARN_OBSTACLE_DETECTOR_IMPROVED_HPP
#define TF2_LEARN_OBSTACLE_DETECTOR_IMPROVED_HPP

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"


namespace tf2_learn
{
    class obs_det_imp_node:public rclcpp::Node
    {
        public:
            obs_det_imp_node(const std::string name);
        private:
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
            void callback_(sensor_msgs::msg::LaserScan::UniquePtr msg_);
            
    };
}

#endif // TF2_LEARN_OBSTACLE_DETECTOR_IMPROVED_HPP