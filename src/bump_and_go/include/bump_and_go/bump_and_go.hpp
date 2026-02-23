#ifndef BUMP_AND_GO__BUMP_AND_GO_HPP
#define BUMP_AND_GO__BUMP_AND_GO_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include<string>

namespace bump_and_go
{
    using namespace std::chrono_literals;

    class bumpandgonode : public rclcpp::Node
    {
        public:
            bumpandgonode(const std::string name);
        
        private:
            //节点基础配置
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
            rclcpp::TimerBase::SharedPtr timer_;

            //订阅消息存储
            sensor_msgs::msg::LaserScan::UniquePtr laser_msg;

            //状态记录
            enum class State
            {
                forward,
                stop,
                back,
                turn
            };

            State state_;
            rclcpp::Time state_ts;
            int turn_direction;

            //固定限制
            const rclcpp::Duration backtime{2s};
            const rclcpp::Duration turntime{2s};
            const rclcpp::Duration scantimeout{1s};

            static constexpr float linear_vel = 3.0f;
            static constexpr float angular_vel = 1.0f;
            static constexpr float obstacle_dist = 1.5f;

            // 实现方法
            void callback(sensor_msgs::msg::LaserScan::UniquePtr msg_);
            void control_cycle();
            void go_state(State state);
            bool forward_to_stop();
            bool forward_to_back();
            bool stop_to_forward();
            bool back_to_turn();
            bool turn_to_forward();
            int decide_turn_direction();
    };
}

#endif