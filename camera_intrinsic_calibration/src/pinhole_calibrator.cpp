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

#include "camera_intrinsic_calibration/pinhole_calibrator.hpp"

namespace camera_intrinsic_calibration
{

PinholeCalibrator::PinholeCalibrator(const YAML::Node & config)
{
  int corner_cols = config["pattern_detector"]["chessboard"]["cols"].as<int>();
  int corner_rows = config["pattern_detector"]["chessboard"]["rows"].as<int>();
  corner_size_ = cv::Size(corner_cols, corner_rows);
  float grid_size = config["pattern_detector"]["chessboard"]["size"].as<float>();
  // generate 3D corner points
  for (int i = 0; i < corner_cols; i++) {
    for (int j = 0; j < corner_rows; j++) {
      pattern_points_.push_back(cv::Point3f(i * grid_size, j * grid_size, 0.0f));
    }
  }
  //
  int image_width = config["image_width"].as<int>();
  int image_height = config["image_hight"].as<int>();
  image_size_ = cv::Size(image_width, image_height);
}

bool PinholeCalibrator::process_image(const cv::Mat & image)
{
  total_img_num_++;
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> image_corners;
  if (!cv::findChessboardCorners(gray_image, corner_size_, image_corners)) {
    status_message_ = "[collect data] failed to find chessborad corners, total image:" +
      std::to_string(total_img_num_);
    return false;
  }
  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::EPS, 50, 0.001);
  cv::cornerSubPix(gray_image, image_corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
  // check
  // if (!image_selector.check(image_corners)) {
  //   return false;
  // }
  // add into
  image_points_.push_back(image_corners);
  object_points_.push_back(pattern_points_);
  valid_img_num_++;
  // debug
  debug_image_ = image;
  cv::drawChessboardCorners(debug_image_, corner_size_, image_corners, true);
  status_message_ = "[collect data] total image:" + std::to_string(total_img_num_) +
    ", valid image:" + std::to_string(valid_img_num_) + ".";
  return true;
}

bool PinholeCalibrator::ready_to_optimize()
{
  return valid_img_num_ >= 15;
}

bool PinholeCalibrator::optimize()
{
  // check min valid num of images
  if (valid_img_num_ < 10) {
    status_message_ =
      "[optimize] failed to optimize, image num at least 10, but:" + std::to_string(valid_img_num_);
    return false;
  }
  double rms = cv::calibrateCamera(
    object_points_, image_points_, image_size_, camera_intrinsic_, camera_distortion_,
    board_rotations_, board_translations_);
  status_message_ = "[optimize] successed to optimize, image:" + std::to_string(valid_img_num_) +
    ", RMS re-projection error:" + std::to_string(rms);
  calibreted_ = true;
  return true;
}

void PinholeCalibrator::clear()
{
  total_img_num_ = 0;
  valid_img_num_ = 0;
  image_points_.clear();
  object_points_.clear();
  calibreted_ = false;
  status_message_ = "";
}

std::string PinholeCalibrator::get_camera_model()
{
  return "pinhole_radtan";
}

std::vector<double> PinholeCalibrator::get_intrinsics()
{
  if (!calibreted_) {
    return {};
  } else {
    return {
      camera_intrinsic_.at<double>(0, 0), camera_intrinsic_.at<double>(1, 1),
      camera_intrinsic_.at<double>(0, 2), camera_intrinsic_.at<double>(1, 2)};
  }
}
std::vector<double> PinholeCalibrator::get_distortion_coeffs()
{
  if (!calibreted_) {
    return {};
  } else {
    return {
      camera_distortion_.at<double>(0, 0), camera_distortion_.at<double>(0, 1),
      camera_distortion_.at<double>(0, 2), camera_distortion_.at<double>(0, 3),
      camera_distortion_.at<double>(0, 4)};
  }
}
const std::string & PinholeCalibrator::get_status_message()
{
  return status_message_;
}

cv::Mat PinholeCalibrator::get_debug_image()
{
  return debug_image_;
}

}  // namespace camera_intrinsic_calibration
