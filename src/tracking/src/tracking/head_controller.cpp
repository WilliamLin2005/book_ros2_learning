#include "tracking/head_controller.hpp"
#include "tracking/pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"

//msg
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tracking_msgs/msg/pan_tilt_command.hpp"

//lifecyclenode
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include<utility>

namespace tracking
{
    using callbackreturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using namespace std::chrono_literals;

    HeadController::HeadController(const std::string name): rclcpp_lifecycle::LifecycleNode(name),
    pan_pid(0.0, 1.0, 0.0, 0.3),
    tilt_pid(0.0, 1.0, 0.0, 0.3)
    {
        error_sub_ = this->create_subscription<tracking_msgs::msg::PanTiltCommand>("error", rclcpp::SensorDataQoS(), [this](tracking_msgs::msg::PanTiltCommand::UniquePtr error_msg)
                                                                                   { this->error_callback(std::move(error_msg)); });
        joint_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>("joint_state", rclcpp::SensorDataQoS(), [this](control_msgs::msg::JointTrajectoryControllerState::UniquePtr joint_msg)
                                                                                                        { this->joint_state_callback(std::move(joint_msg)); });
        joint_command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_state_command", 10);
        //这里千万不能定义timer_,否则节点刚创建完就会开始循环控制,发送控制指令
    }

    void HeadController::error_callback(tracking_msgs::msg::PanTiltCommand::UniquePtr error_msg)
    {
        last_error = std::move(error_msg);
    }

    void HeadController::joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr joint_state_msg)
    {
        last_state = std::move(joint_state_msg);
    }

    callbackreturn HeadController::on_configure(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(get_logger(), "headcontroller configured");
        pan_pid.set_pid(0.4, 0.05, 0.55);
        tilt_pid.set_pid(0.4, 0.05, 0.55);

        return callbackreturn::SUCCESS;
    }

    callbackreturn HeadController::on_activate(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(get_logger(),"headcontroller activated");

        //启动publisher
        joint_command_pub_->on_activate();
        timer_ = this->create_wall_timer(50ms, [this]()
                                         { this->timer_callback(); });

        return callbackreturn::SUCCESS;
    }

    callbackreturn HeadController::on_deactivate(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO(get_logger(), "headcontroller deactivated");
        // 移到初始点
        trajectory_msgs::msg::JointTrajectory move_to_start;
        move_to_start.points.resize(1);
        move_to_start.header.set__stamp(now());
        move_to_start.joint_names = last_state->joint_names;
        move_to_start.points[0].velocities.resize(2);
        move_to_start.points[0].accelerations.resize(2);
        move_to_start.points[0].positions.resize(2);
        move_to_start.points[0].velocities[0] = 0.0;
        move_to_start.points[0].velocities[1] = 0.0;
        move_to_start.points[0].accelerations[0] = 0.0;
        move_to_start.points[0].accelerations[1] = 0.0;
        move_to_start.points[0].positions[0] = 0.0;
        move_to_start.points[0].positions[1] = 0.0;

        joint_command_pub_->publish(move_to_start);
        joint_command_pub_->on_deactivate();
        timer_ = nullptr;

        return callbackreturn::SUCCESS;
    }

    void HeadController::timer_callback()
    {
        if(last_state==nullptr)
        {
            RCLCPP_INFO(get_logger(), "no information from joint state");
            return;
        }

        trajectory_msgs::msg::JointTrajectory joint_command_msg;
        joint_command_msg.header.stamp = now();
        joint_command_msg.joint_names = last_state->joint_names;
        joint_command_msg.points.resize(1);
        joint_command_msg.points[0].velocities.resize(2);
        joint_command_msg.points[0].accelerations.resize(2);
        joint_command_msg.points[0].positions.resize(2);

        if(last_error==nullptr || (this->now()-last_error->timestamp)>1s)
        {
        joint_command_msg.points[0].velocities[0] = 0.0;
        joint_command_msg.points[0].velocities[1] = 0.0;
        joint_command_msg.points[0].accelerations[0] = 0.0;
        joint_command_msg.points[0].accelerations[1] = 0.0;
        joint_command_msg.points[0].positions[0] = 0.0;
        joint_command_msg.points[0].positions[1] = 0.0;

        joint_command_pub_->publish(joint_command_msg);
        return;
        }
        else
        {
            double pan_error = pan_pid.get_output(last_error->pan);
            double tilt_error = tilt_pid.get_output(last_error->tilt);

            joint_command_msg.points[0].positions[0] = last_state->actual.positions[0]-pan_error;
            joint_command_msg.points[0].positions[1] = last_state->actual.positions[1]-tilt_error;
            joint_command_msg.points[0].velocities[0] = 1.0;
            joint_command_msg.points[0].velocities[1] = 1.0;
            joint_command_msg.points[0].accelerations[0] = 1.0;
            joint_command_msg.points[0].accelerations[1] = 1.0;

            joint_command_pub_->publish(joint_command_msg);
            return;
        }
        }
}