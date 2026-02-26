#ifndef VFF_AVOIDANCE__VFF_AVOIDANCE_HPP_
#define VFF_AVOIDANCE__VFF_AVOIDANCE_HPP_

#include <memory>
#include <vector>

#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"visualization_msgs/msg/marker_array.hpp"

namespace vff_avoidance
{
    struct vff_vector
    {
        std::vector<float> attractive_vector;
        std::vector<float> repulsive_vector;
        std::vector<float> result_vector;
    };

    enum class vff_colour
    {
        BLUE,
        RED,
        GREEN
    };

    class vff_avoidance_node:public rclcpp::Node{
        public:
            vff_avoidance_node(const std::string name);
        
        protected:
            vff_vector get_vff(const sensor_msgs::msg::LaserScan &laser_msg);
            visualization_msgs::msg::MarkerArray get_vff_debug(const vff_vector &vectors);
            visualization_msgs::msg::Marker make_marker(const std::vector<float> &vector_, vff_colour colour_);

        private:
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;
            rclcpp::TimerBase::SharedPtr timer_;
            void laser_sub_callback(sensor_msgs::msg::LaserScan::UniquePtr msg_);
            void timer_callback();
            sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
    };
}

#endif  // VFF_AVOIDANCE__VFF_AVOIDANCE_HPP_    