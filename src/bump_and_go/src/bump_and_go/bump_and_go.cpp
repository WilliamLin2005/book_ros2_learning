#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"
#include"bump_and_go/bump_and_go.hpp"
#include<utility>

namespace bump_and_go
{
    using namespace std::chrono_literals;

    bumpandgonode::bumpandgonode(const std::string name):Node(name),state_(State::forward),turn_direction(1)
    {
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_input", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::UniquePtr mes)
                                                                           { callback(std::move(mes)); });
        twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("twist_out", 10);
        timer_ = this->create_wall_timer(50ms, [this]()
                                         { control_cycle(); });
        state_ts = now();
    }

    void bumpandgonode::callback(sensor_msgs::msg::LaserScan::UniquePtr msg_)
    {
        laser_msg = std::move(msg_);
    }

    void bumpandgonode::control_cycle()
    {
        if(laser_msg==nullptr)
            return;

        geometry_msgs::msg::Twist out_vel;

        switch (state_)
        {
            case State::forward:
                out_vel.linear.set__x(linear_vel);

                if(forward_to_back()){
                    go_state(State::back);
                }

                if(forward_to_stop()){
                    go_state(State::stop);
                }

                break;

            case State::stop:
                out_vel.linear.set__x(0);

                if(stop_to_forward()){
                    go_state(State::forward);
                }

                break;

            case State::back:
                out_vel.linear.set__x(-linear_vel);

                if(back_to_turn()){
                    go_state(State::turn);
                }

                break;

            case State::turn:
                out_vel.angular.set__z(turn_direction * angular_vel);

                if(turn_to_forward()){
                    go_state(State::forward);
                }
            }

            twist_pub->publish(out_vel);
    }

    void bumpandgonode::go_state(State state)
    {
        state_ = state;
        state_ts = now();

        if(state==State::turn)
        {
            turn_direction = bumpandgonode::decide_turn_direction();
        }
    }

    bool bumpandgonode::forward_to_stop()
    {
        auto elapsed = now() - rclcpp::Time(laser_msg->header.stamp);
        return elapsed >= scantimeout;
    }

    bool bumpandgonode::forward_to_back()
    {
        auto pos = laser_msg->ranges.size() / 2;
        return laser_msg->ranges[pos] < obstacle_dist;
    }

    bool bumpandgonode::stop_to_forward()
    {
        auto elapsed = now() - rclcpp::Time(laser_msg->header.stamp);
        return elapsed < scantimeout;
    }

    bool bumpandgonode::back_to_turn()
    {
        auto elapsed = now() - state_ts;
        return elapsed > backtime;
    }

    bool bumpandgonode::turn_to_forward()
    {
        auto elapsed = now() - state_ts;
        return elapsed > turntime;
    }

    int bumpandgonode::decide_turn_direction()
    {
        auto left = laser_msg->ranges.size() *3 / 4;
        auto right = laser_msg->ranges.size() / 4;

        if(laser_msg->ranges[left] <= laser_msg->ranges[right])
        {
            return -1;
        }
        else
        {
            return 1;
        }
    }
}