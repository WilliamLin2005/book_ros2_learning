#ifndef TF2_LEARN__OBSTACLE_MONITOR_HPP
#define TF2_LEARN__OBSTACLE_MONITOR_HPP

#include<memory>
#include"rclcpp/rclcpp.hpp"
#include"tf2_ros/buffer.h"
#include"tf2_ros/transform_listener.h"
#include"visualization_msgs/msg/marker.hpp"

namespace tf2_learn
{
    class obs_mon_node:public rclcpp::Node
    {
        public:
            obs_mon_node(const std::string name);
        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::TimerBase::SharedPtr timer_;
            void control_cycle();
    };
}
//namespace tf2_learn
#endif 