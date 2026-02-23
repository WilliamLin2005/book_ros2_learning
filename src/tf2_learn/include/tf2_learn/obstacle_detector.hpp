#ifndef TF2_LEARN_OBSTACLE_DETECTOR_HPP_
#define TF2_LEARN_OBSTACLE_DETECTOR_HPP_

#include<memory>
#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"tf2_ros/static_transform_broadcaster.hpp"

namespace tf2_learn
{
    class obs_det_node :public rclcpp::Node
    {
        public:
            obs_det_node(const std::string name);
        private:
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            void callback_(sensor_msgs::msg::LaserScan::UniquePtr msg_);
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    };
}
// namespace tf2_learn
#endif  // TF2_LEARN_OBSTACLE_DETECTOR_HPP_