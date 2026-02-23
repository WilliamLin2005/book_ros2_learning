#include"tf2_learn/obstacle_detector.hpp"
#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/transform_stamped.hpp"
#include<utility>

namespace tf2_learn
{
    obs_det_node::obs_det_node(const std::string name): Node(name)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::SensorDataQoS(),
                                                                            [this](sensor_msgs::msg::LaserScan::UniquePtr msg_)
                                                                            { this->callback_(std::move(msg_)); });
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    void obs_det_node::callback_(sensor_msgs::msg::LaserScan::UniquePtr msg_)
    {
        double dist = msg_->ranges[msg_->ranges.size() / 2];

        if(std::isfinite(dist))
        {
            geometry_msgs::msg::TransformStamped tf_transform_;

            tf_transform_.header = msg_->header;
            tf_transform_.child_frame_id = "obstacle_det";
            tf_transform_.transform.translation.set__x(dist);

            tf_broadcaster_->sendTransform(tf_transform_);
        }
    }
}