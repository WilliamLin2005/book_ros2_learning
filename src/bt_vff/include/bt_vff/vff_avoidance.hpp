#ifndef BT_VFF__VFF_AVOIDANCE_HPP_
#define BT_VFF__VFF_AVOIDANCE_HPP_

#include<string>

//behavior tree
#include"behaviortree_cpp_v3/behavior_tree.h"
#include"behaviortree_cpp_v3/bt_factory.h"

//ros2
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"visualization_msgs/msg/marker_array.hpp"
#include"rclcpp/rclcpp.hpp"

namespace bt_vff
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



    class vff_avoidance:public BT::ActionNodeBase
    {
        public:
            explicit vff_avoidance(const std::string &xml_pkg_name, const BT::NodeConfiguration &conf);
            void halt() ;
            BT::NodeStatus tick();

            static BT::PortsList providedPorts()
            {
                return BT::PortsList({});
            }

            vff_vector get_vff(const sensor_msgs::msg::LaserScan &laser_msg);
            visualization_msgs::msg::MarkerArray get_vff_debug(const vff_vector &vectors);
            visualization_msgs::msg::Marker make_marker(const std::vector<float> &vector_, vff_colour colour_);

        
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;
            sensor_msgs::msg::LaserScan::UniquePtr latest_scan;
    };

}
#endif //BT_VFF__VFF_AVOIDANCE_HPP_



