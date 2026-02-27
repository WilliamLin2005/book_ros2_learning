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

        const vff_vector &vffvec = get_vff(*last_scan_);

        geometry_msgs::msg::Twist vel_twist;
        const auto &result = vffvec.result_vector;
        double angle = atan2(result[1], result[0]);
        vel_twist.linear.set__x(std::clamp(result[0], 0.0f, 3.0f));
        vel_twist.angular.set__z(std::clamp(angle, -0.5, 0.5));

        const visualization_msgs::msg::MarkerArray &markerarr = get_vff_debug(vffvec);

        twist_pub->publish(vel_twist);

        marker_array_pub->publish(markerarr);
        
    }

    vff_vector vff_avoidance_node::get_vff(const sensor_msgs::msg::LaserScan &laser_msg)
    {
        const float OBS_DIST_MAX = 1.5f; // 只考虑 1.5 米内的障碍物
        const float K_REPULSIVE = 0.05f; // 斥力增益系数（关键！控制排斥的猛烈程度）

        vff_vector vff_;
        vff_.attractive_vector = {1.0f, 0.0f}; // 保持恒定向前的引力
        vff_.repulsive_vector = {0.0f, 0.0f};
        vff_.result_vector = {0.0f, 0.0f};

        for (size_t i = 0; i < laser_msg.ranges.size(); i++)
        {
            float dist = laser_msg.ranges[i];
            
            // 必须过滤无穷大和 NaN，否则一个坏点毁掉整个向量
            if (std::isfinite(dist) && dist < OBS_DIST_MAX && dist > 0.05f)
            {
                float angle = laser_msg.angle_min + laser_msg.angle_increment * i;
                float opposite_angle = angle + M_PI; // 指向相反方向
                
                // 使用反比模型：越近，排斥力成倍剧增！
                float force_magnitude = K_REPULSIVE * (1.0f / dist); 

                // 向量累加（不需要除以 times）
                vff_.repulsive_vector[0] += force_magnitude * cos(opposite_angle);
                vff_.repulsive_vector[1] += force_magnitude * sin(opposite_angle);
            }
        }

        // 合力 = 引力 + 斥力
        vff_.result_vector[0] = vff_.attractive_vector[0] + vff_.repulsive_vector[0];
        vff_.result_vector[1] = vff_.attractive_vector[1] + vff_.repulsive_vector[1];

        return vff_;
    }

    visualization_msgs::msg::MarkerArray 
    vff_avoidance_node::get_vff_debug(const vff_vector &vectors)
    {
        visualization_msgs::msg::MarkerArray marker_arr;

        marker_arr.markers.push_back(make_marker(vectors.attractive_vector, vff_colour::BLUE));
        marker_arr.markers.push_back(make_marker(vectors.repulsive_vector, vff_colour::RED));
        marker_arr.markers.push_back(make_marker(vectors.result_vector, vff_colour::GREEN));

        return marker_arr;
    }

    visualization_msgs::msg::Marker vff_avoidance_node::make_marker(const std::vector<float> &vector_, vff_colour colour_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = now();
        marker.set__type(visualization_msgs::msg::Marker::ARROW);
        marker.set__action(visualization_msgs::msg::Marker::ADD);

        geometry_msgs::msg::Point start_;
        start_.set__x(0);
        start_.set__y(0);
        start_.set__z(0);

        geometry_msgs::msg::Point end_;
        end_.set__x(vector_[0]);
        end_.set__y(vector_[1]);
        end_.set__z(0);

        marker.points = {start_, end_};

        marker.scale.set__x(0.04);
        marker.scale.set__y(0.08);
        marker.scale.set__z(0.1);

        switch (colour_)
        {
        case vff_colour::RED:
            marker.set__id(0);
            marker.color.r = 1.0;
            break;
        case vff_colour::BLUE:
            marker.set__id(1);
            marker.color.b = 1.0;
            break;
        case vff_colour::GREEN:
            marker.set__id(2);
            marker.color.g = 1.0;
        }
        marker.color.a = 1.0;

        return marker;
    }
}