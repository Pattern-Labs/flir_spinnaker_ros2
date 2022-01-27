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

#include <flir_spinnaker_ros2/camera_autoexposure.h>

#include <fstream>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "logging.h"

namespace flir_spinnaker_ros2
{

constexpr uint32_t CameraAutoExposure::kMinExposureUs;
constexpr uint32_t CameraAutoExposure::kMaxExposureUs;
constexpr float CameraAutoExposure::kMaxGain;
constexpr uint32_t CameraAutoExposure::kExposureTransition;
constexpr float CameraAutoExposure::kGainTransition;

CameraAutoExposure::CameraAutoExposure(const rclcpp::NodeOptions& options) 
: Node("cam_autoexposure", options)
{

  declare_parameter("camera_name", "blackfly_s");
  get_parameter("camera_name", camera_name_);
  declare_parameter("target_intensity", 120);
  get_parameter("target_intensity", target_intensity_);
  declare_parameter("intensity_update_gain", 0.5);
  get_parameter("intensity_update_gain", intensity_update_gain_);
  camera_meta_sub_ = create_subscription<image_meta_msgs_ros2::msg::ImageMetaData>(
      "/" + camera_name_ + "/meta", qosDepth_, std::bind(&CameraAutoExposure::metaCallback, this, std::placeholders::_1));
  control_pub_ = 
    create_publisher<camera_control_msgs_ros2::msg::CameraControl>("/" + camera_name_ + "/control", qosDepth_);
}

CameraAutoExposure::~CameraAutoExposure() {

}

void CameraAutoExposure::metaCallback(
  const image_meta_msgs_ros2::msg::ImageMetaData::ConstSharedPtr msg) {
  if (msg->brightness == -1) {
    RCLCPP_WARN(get_logger(), "Brightness is disabled, not running exposure control.");
    return;
  }

  // Filter the intensity with a single pole IIR based on the coef configured.
  const int16_t new_intensity = msg->brightness;
  const double filtered_intensity = ((1.0-intensity_update_gain_)*intensity_accum_) + (intensity_update_gain_ * new_intensity);
  intensity_accum_ = filtered_intensity;

  const uint32_t current_exposure_us = msg->exposure_time;
  const double combined_exposure_gain = current_exposure_us * std::pow(10, msg->gain / 20.0);

  // Compute the multiplier to apply to the current exposure time that would result in the desired intensity.
  const double target_exposure_gain_multiplier = target_intensity_ / filtered_intensity;

  const double target_combined_exposure_gain = combined_exposure_gain * target_exposure_gain_multiplier;

  camera_control_msgs_ros2::msg::CameraControl camera_control_msg;
  camera_control_msg.header.stamp = now();
  camera_control_msg.header.frame_id = msg->header.frame_id;

  const uint32_t max_exposure_time = std::min(msg->max_exposure_time, kMaxExposureUs);
  if (target_combined_exposure_gain < kExposureTransition) {
    camera_control_msg.exposure_time = target_combined_exposure_gain;
    camera_control_msg.gain = 1.0;
  } else if (target_combined_exposure_gain < (kExposureTransition * std::pow(10, kGainTransition / 20.0))) {
    camera_control_msg.exposure_time = kExposureTransition;
    const double target_linear_gain = target_combined_exposure_gain / kExposureTransition;
    camera_control_msg.gain = 20 * std::log10(target_linear_gain);
  } else if (target_combined_exposure_gain < (max_exposure_time * std::pow(10, kGainTransition / 20.0))) {
    camera_control_msg.gain = kGainTransition;
    const double target_exposure = target_combined_exposure_gain / std::pow(10, kGainTransition / 20.0);
    camera_control_msg.exposure_time = static_cast<uint32_t>(std::round(target_exposure));
  } else if (target_combined_exposure_gain < (max_exposure_time * std::pow(10, kMaxGain / 20.0))) {
    camera_control_msg.exposure_time = max_exposure_time;
    const double target_linear_gain = target_combined_exposure_gain / max_exposure_time;
    camera_control_msg.gain = 20 * std::log10(target_linear_gain);
  } else {
    camera_control_msg.gain = kMaxGain;
    camera_control_msg.exposure_time = max_exposure_time;
  }

  RCLCPP_DEBUG_STREAM(
    get_logger(), "Computed filtered intensity "
                    << filtered_intensity << "Setting exposure to "
                    << camera_control_msg.exposure_time << " gain to "
                    << camera_control_msg.gain);
  control_pub_->publish(camera_control_msg);
}

void CameraAutoExposure::run() {

}

}  // namespace flir_spinnaker_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(flir_spinnaker_ros2::CameraAutoExposure)
