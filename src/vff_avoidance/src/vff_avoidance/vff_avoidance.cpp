#include<memory>
#include<vector>
#include<algorithm>
#include<utility>

#include"vff_avoidance/vff_avoidance.hpp"
#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"visualization_msgs/msg/marker_array.hpp"


namespace vff_avoidance
{
    using namespace std::chrono_literals;

    vff_avoidance_node::vff_avoidance_node(const std::string name):Node(name)
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::SensorDataQoS(),
                                                                           [this](sensor_msgs::msg::LaserScan::UniquePtr msg_)
                                                                           { this->laser_sub_callback(std::move(msg_));});
        twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("output_twist", 100);
        marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("output_vff_debug", 100);
        timer_ = this->create_wall_timer(50ms, [this]()
                                            { timer_callback(); });
    }

    void vff_avoidance_node::laser_sub_callback(sensor_msgs::msg::LaserScan::UniquePtr msg_)
    {
        last_scan_ = std::move(msg_);
    }

    void vff_avoidance_node::timer_callback()
    {
        if(last_scan_==nullptr or now()-last_scan_->header.stamp>1s)
        {
            return;
        }
    }
}