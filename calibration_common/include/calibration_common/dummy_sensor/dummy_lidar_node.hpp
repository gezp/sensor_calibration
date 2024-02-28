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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "calibration_common/publisher/cloud_publisher.hpp"

namespace calibration_common
{

class DummyLidarNode
{
public:
  explicit DummyLidarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  bool read_data();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CloudPublisher> cloud_pub_;
  // param
  std::string frame_id_{"lidar"};
  std::string data_dir_;
  double rate_{10};
  double timestamp_{0};
  bool loop_{true};
  // data
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointclouds_;
  size_t current_idx_{0};
};
}  // namespace calibration_common
