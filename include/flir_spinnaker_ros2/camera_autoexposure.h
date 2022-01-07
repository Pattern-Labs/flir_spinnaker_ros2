// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_
#define FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_

#include <camera_control_msgs_ros2/msg/camera_control.hpp>
#include <image_meta_msgs_ros2/msg/image_meta_data.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace flir_spinnaker_ros2
{
class CameraAutoExposure : public rclcpp::Node
{
public:
  explicit CameraAutoExposure(const rclcpp::NodeOptions & options);
  ~CameraAutoExposure();

private:

  void run();  // thread

  // void imageCallback(
  //   const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void metaCallback(const image_meta_msgs_ros2::msg::ImageMetaData::ConstSharedPtr msg);

  // rcl_interfaces::msg::SetParametersResult parameterChanged(
  //   const std::vector<rclcpp::Parameter> & params);

  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
  //   image_sub_;
  rclcpp::Subscription<image_meta_msgs_ros2::msg::ImageMetaData>::SharedPtr
    camera_meta_sub_;
  rclcpp::Publisher<camera_control_msgs_ros2::msg::CameraControl>::SharedPtr
    control_pub_;

  bool debug_{false};
  std::string camera_name_;
  uint32_t current_exposure_time_{0};
  float current_gain_{std::numeric_limits<float>::lowest()};
  int16_t target_intensity_;

  static constexpr uint32_t kMinExposureUs = 1000;
  static constexpr uint32_t kMaxExposureUs = 30000;
  static constexpr float    kMaxGain = 40.0;
  static constexpr uint32_t kExposureTransition = 20000;
  static constexpr float    kGainTransition = 15.0;

  double intensity_accum_ = 0.0;
  // Bias on the IIR, range is 0.-1. where 1.0 would immediately replace the accum with the current value (ie no filtering).
  double intensity_update_gain_ = 1.0;

  // sensor_msgs::msg::Image::ConstSharedPtr imageMsg_;
  image_meta_msgs_ros2::msg::ImageMetaData::ConstSharedPtr metaMsg_;

  // rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
  //   callbackHandle_;  
  bool exposure_running_{false};
  std::shared_ptr<std::thread> thread_;
  bool should_run_{true};
  // std::vector<std::string> parameterList_;  // remember original ordering
  int qosDepth_{4};
};
}  // namespace flir_spinnaker_ros2
#endif  // FLIR_SPINNAKER_ROS2__CAMERA_DRIVER_H_
