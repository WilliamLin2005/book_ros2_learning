#ifndef TRACKING__HEAD_CONTROLLER_HPP
#define TRACKING__HEAD_CONTROLLER_HPP

//pid
#include "tracking/pid_controller.hpp"

//msg
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tracking_msgs/msg/pan_tilt_command.hpp"

//lifecyclenode
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace tracking
{
    using callbackreturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HeadController:public rclcpp_lifecycle::LifecycleNode
    {
        public:
            HeadController(const std::string name);

        private:
            rclcpp::Subscription<tracking_msgs::msg::PanTiltCommand>::SharedPtr error_sub_;
            rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_state_sub_;
            rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_pub_;     
            rclcpp::TimerBase::SharedPtr timer_;

            tracking_msgs::msg::PanTiltCommand::UniquePtr last_error;
            control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state;
            pidcontroller pan_pid, tilt_pid;

            void timer_callback();
            void error_callback(tracking_msgs::msg::PanTiltCommand::UniquePtr error_msg);
            void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr joint_state_msg);

        protected:
            callbackreturn on_configure(const rclcpp_lifecycle::State &prev_state);
            callbackreturn on_activate(const rclcpp_lifecycle::State &prev_state);
            callbackreturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
    };
} // namespace tracking

#endif