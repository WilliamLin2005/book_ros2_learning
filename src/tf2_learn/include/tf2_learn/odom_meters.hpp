#ifndef TF2_LEARN_ODOM_METERS_HPP
#define TF2_LEARN_ODOM_METERS_HPP

#include"rclcpp/rclcpp.hpp"
#include"tf2_ros/buffer.h"
#include"tf2_ros/transform_listener.h"
#include<memory>

namespace tf2_learn
{
    class odom_meters_node:public rclcpp::Node
    {
        public:
            odom_meters_node(const std::string name);
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            std::shared_ptr<tf2_ros::Buffer> buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            double curr_x;
            double curr_y;
            bool is_first_time;
            void callback();
    };
}

#endif //TF2_LEARN_ODOM_METERS_HPP