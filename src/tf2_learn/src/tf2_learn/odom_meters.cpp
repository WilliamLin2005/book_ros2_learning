#include<memory>
#include"tf2_learn/odom_meters.hpp"

//tf2
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

//ros2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace tf2_learn
{
    using namespace std::chrono_literals;

    odom_meters_node::odom_meters_node(const std::string name):Node(name),is_first_time(true)
    {
        timer_ = this->create_wall_timer(1s, [this]()
                                         { this->callback(); });
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

    void odom_meters_node::callback()
    {
        if(is_first_time)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing odometry baseline...");
            try
            {
                auto tf=buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero, 100ms);
                curr_x = tf.transform.translation.x;
                curr_y = tf.transform.translation.y;
                is_first_time = false;
            }
            catch(tf2::TransformException & ex)
            {
                return;
            }
            return;
        }

        try
        {     
            geometry_msgs::msg::TransformStamped odom_to_base_footprint =
            buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero, 100ms);
            double base_x = odom_to_base_footprint.transform.translation.x;
            double base_y = odom_to_base_footprint.transform.translation.y;
            double dx = base_x - curr_x;
            double dy = base_y - curr_y;
            double move_dist = std::hypot(dx, dy);

            RCLCPP_INFO_STREAM(this->get_logger(), "moved distance: " << move_dist);
            curr_x = base_x;
            curr_y = base_y;
        }
        catch(tf2::TransformException & ex)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "transform odom to robot not found: " << ex.what());
            return;
        }

        
    }
}