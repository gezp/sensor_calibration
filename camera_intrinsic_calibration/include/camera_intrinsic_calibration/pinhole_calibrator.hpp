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

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <string>

#include "opencv2/opencv.hpp"

namespace camera_intrinsic_calibration
{

class PinholeCalibrator
{
public:
  explicit PinholeCalibrator(const YAML::Node & config);
  bool process_image(const cv::Mat & image);
  bool ready_to_optimize();
  bool optimize();
  void clear();
  // get result
  std::string get_camera_model();
  std::vector<double> get_intrinsics();
  std::vector<double> get_distortion_coeffs();
  const std::string & get_status_message();
  cv::Mat get_debug_image();

private:
  // param
  cv::Size corner_size_;
  cv::Size image_size_;
  // 3d calibration pattern points in a board
  std::vector<cv::Point3f> pattern_points_;
  // 2d point of corners in each chessboard
  std::vector<std::vector<cv::Point2f>> image_points_;
  // 3d point of corners in each chessboard
  std::vector<std::vector<cv::Point3f>> object_points_;
  //
  int total_img_num_{0};
  int valid_img_num_{0};
  // calibration result
  bool calibreted_{false};
  cv::Mat camera_intrinsic_;
  cv::Mat camera_distortion_;
  std::vector<cv::Mat> board_rotations_;
  std::vector<cv::Mat> board_translations_;
  // message
  std::string status_message_;
  cv::Mat debug_image_;
};

}  // namespace camera_intrinsic_calibration
