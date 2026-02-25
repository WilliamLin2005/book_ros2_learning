#include<utility>
#include <memory>
#include <cmath> // 用于 std::isfinite

//tf2
#include "tf2/transform_datatypes.h" // 用于 tf2::Transform 和 tf2::Quaternion
#include "tf2/LinearMath/Quaternion.h" // 用于 tf2::Quaternion
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 用于 tf2::toMsg 和 tf2::fromMsg

//ros2
#include "tf2_learn/obstacle_detector_improved.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tf2_learn
{
    using namespace std::chrono_literals;

    obs_det_imp_node::obs_det_imp_node(const std::string name):Node(name)
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_input", rclcpp::SensorDataQoS(),
                                                                            [this](sensor_msgs::msg::LaserScan::UniquePtr msg_)
                                                                            { this->callback_(std::move(msg_)); });
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    void obs_det_imp_node::callback_(sensor_msgs::msg::LaserScan::UniquePtr msg_)
    {
        double dist = msg_->ranges[msg_->ranges.size() / 2];

        if(isfinite(dist))
        {
            tf2::Transform laser_to_obs;
            laser_to_obs.setOrigin(tf2::Vector3(dist,0,0));
            laser_to_obs.setRotation(tf2::Quaternion(0, 0, 0, 1));

            geometry_msgs::msg::TransformStamped odom_to_laser_msg;
            tf2::Transform odom_to_laser;
            try
            {
                odom_to_laser_msg = buffer_->lookupTransform("odom", "base_laser_link", msg_->header.stamp, 100ms);
                tf2::fromMsg(odom_to_laser_msg.transform, odom_to_laser);
            }
            catch(tf2::TransformException &ex)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "transform odom_to_laser not found: " << ex.what());
                return;
            }

            tf2::Transform odom_to_obs = odom_to_laser * laser_to_obs;
            geometry_msgs::msg::TransformStamped odom_to_obs_msg;
            odom_to_obs_msg.transform = tf2::toMsg(odom_to_obs);
            odom_to_obs_msg.header.frame_id = "odom";
            odom_to_obs_msg.child_frame_id = "obs_det";
            odom_to_obs_msg.header.stamp = msg_->header.stamp;

            tf_broadcaster_->sendTransform(odom_to_obs_msg);
        }
    }
}