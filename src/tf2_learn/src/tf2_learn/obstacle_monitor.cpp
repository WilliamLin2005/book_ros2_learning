#include<memory>
#include"tf2_learn/obstacle_monitor.hpp"
//tf2
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"//重载了转换函数，让 TF2 库能够识别并直接填入 ROS 2 的消息格式
//ros2
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tf2_learn
{
    using namespace std::chrono_literals;

    obs_mon_node::obs_mon_node(const std::string name):Node(name)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_pub", 1);
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        timer_ = this->create_wall_timer(500ms, [this]()
                                         { this->control_cycle(); });
    }

    void obs_mon_node::control_cycle()
    {
        geometry_msgs::msg::TransformStamped robot_to_obs;

        try
        {
            robot_to_obs = buffer_->lookupTransform("base_footprint", "obstacle_det", tf2::TimePointZero);
        }
        catch(const tf2::TransformException & ex)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "robot to obs transformation not found because: " << ex.what());
        }

        double x = robot_to_obs.transform.translation.x;
        double y = robot_to_obs.transform.translation.y;
        double z = robot_to_obs.transform.translation.z;
        double theta_ = atan2(y, x);
        RCLCPP_INFO_STREAM(this->get_logger(),"obs detected at X:"<<x<<" Y: "<<y<<" Z: "<<z<<" ,with atan2: "<<theta_);

        visualization_msgs::msg::Marker marker_;
        marker_.header.frame_id = "base_footprint";
        marker_.header.stamp = now();
        marker_.lifetime = rclcpp::Duration(1s);
        marker_.type = visualization_msgs::msg::Marker::ARROW;
        marker_.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start_;
        geometry_msgs::msg::Point end_;
        end_.set__x(x);end_.set__y(y);end_.set__z(z);
        marker_.points = {start_, end_};

        marker_.color.a = 1.0;
        marker_.color.r = 1.0;
        marker_.scale.set__x(0.1);
        marker_.scale.set__y(0.2);
        marker_.scale.set__z(0.5);

        marker_pub_->publish(marker_);
    }
}
//namespace tf2_learn