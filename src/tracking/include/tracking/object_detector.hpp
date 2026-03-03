// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRACKING__OBJECTDETECTOR_HPP_
#define TRACKING__OBJECTDETECTOR_HPP_

#include <memory>
#include <vector>

#include "vision_msgs/msg/detection2_d.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tracking
{

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector(const std::string node_name);

  //这里使用constSharedPtr &,是因为需要支持多个节点同时处理同一张图。
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

private:
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;

  // HSV ranges for detection [h - H] [s - S] [v - V]
  //这里设置为所有的颜色的值,当需要特定的值时,可以在parameters里面生命
  std::vector<double> hsv_filter_ranges_ {0, 180, 0, 255, 0, 255};
  bool debug_ {true};
};

}  // namespace tracking

#endif  // TRACKING__OBJECTDETECTOR_HPP_
