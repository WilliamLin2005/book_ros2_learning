#include "tracking/object_detector.hpp"
#include "tracking_msgs/msg/pan_tilt_command.hpp"
#include "tracking/head_controller.hpp"
#include "rclcpp/rclcpp.hpp"    


int main(int argc,char*argv[])
{
  rclcpp::init(argc, argv);

  auto transfer_error_node = rclcpp::Node::make_shared("transfer_error_node");
  auto object_detector_node = std::make_shared<tracking::ObjectDetector>("object_detector_node");
  auto headcontroller_node = std::make_shared<tracking::HeadController>("headcontroler_node");

  const int IMG_WIDTH = 640;
  const int IMG_HEIGHT = 480;

  auto error_pub = transfer_error_node->create_publisher<tracking_msgs::msg::PanTiltCommand>(
    "error", 100);
  auto detection_sub = transfer_error_node->create_subscription<vision_msgs::msg::Detection2D>(
    "detection", rclcpp::SensorDataQoS(),
    //订阅检测结果，并将其转换为pan和tilt命令发布到/command话题
    [error_pub,transfer_error_node](vision_msgs::msg::Detection2D::SharedPtr msg) {
      tracking_msgs::msg::PanTiltCommand command;
      //从检测结果中提取目标中心位置，并将其通过以下顺序:[0,1]->[0,2]->[-1,1]转换为pan和tilt命令
      command.pan = (msg->bbox.center.position.x / IMG_WIDTH) * 2.0 - 1.0;
      command.tilt = (msg->bbox.center.position.y / IMG_HEIGHT) * 2.0 - 1.0;
      command.timestamp = transfer_error_node->now();
      error_pub->publish(command);
    });

  rclcpp::executors::SingleThreadedExecutor exec_;
  exec_.add_node(object_detector_node);
  exec_.add_node(transfer_error_node);
  exec_.add_node(headcontroller_node->get_node_base_interface());
  headcontroller_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  exec_.spin();

  rclcpp::shutdown();
  return 0;
}