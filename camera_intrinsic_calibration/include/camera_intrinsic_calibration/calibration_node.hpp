// Copyright 2024 Gezp (https://github.com/gezp).
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

#pragma once

#include <thread>
#include <memory>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "calibration_common/publisher/image_publisher.hpp"
#include "calibration_common/subscriber/image_subscriber.hpp"
#include "calibration_common/calibration_data.hpp"
#include "camera_intrinsic_calibration/pinhole_calibrator.hpp"
#include "calibration_interfaces/msg/calibration_status.hpp"
#include "calibration_interfaces/msg/calibration_command.hpp"

namespace camera_intrinsic_calibration
{

class CalibrationNode
{
  struct SensorData
  {
    double time;
    cv::Mat image;
  };

public:
  explicit CalibrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CalibrationNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void read_data();
  void clear_data();
  void process_command(uint8_t command);
  void update_status_msg(uint8_t state, const std::string & info, bool success = false);
  void save_result();
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<calibration_common::ImageSubscriber> image_sub_;
  rclcpp::Subscription<calibration_interfaces::msg::CalibrationCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<calibration_interfaces::msg::CalibrationStatus>::SharedPtr status_pub_;
  std::shared_ptr<calibration_common::ImagePublisher> debug_image_pub_;
  std::shared_ptr<PinholeCalibrator> calibrator_;
  // param
  std::string frame_id_{"camera"};
  std::string output_file_;
  bool enable_compressed_image_{false};
  bool autostart_{false};
  // calibration flow thread
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<SensorData> sensor_data_buffer_;
  std::shared_ptr<calibration_common::CalibrationData> calibration_data_;
  uint8_t state_;
  bool success_{false};
  bool is_auto_mode_{true};
  bool need_collect_once_{false};
  bool need_optimize_once_{false};
  calibration_interfaces::msg::CalibrationStatus status_msg_;
  std::mutex status_mutex_;
  //
  std::deque<calibration_interfaces::msg::CalibrationCommand> command_msgs_;
  std::mutex command_mutex_;
};
}  // namespace camera_intrinsic_calibration
