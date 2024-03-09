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

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <fstream>

#include "calibration_common/calibration_data.hpp"

namespace calibration_common
{

std::string to_string(const std::vector<double> & data)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < data.size(); i++) {
    ss << data[i];
    if (i + 1 != data.size()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

bool CalibrationData::add_camera_intrinsic_data(
  const std::string & frame_id, const std::string & camera_model_type,
  const std::vector<double> & intrinsics, const std::vector<double> & distortion_coeffs)
{
  CameraIntrinsicData data;
  data.camera_model_type = camera_model_type;
  data.intrinsics = intrinsics;
  data.distortion_coeffs = distortion_coeffs;
  camera_intrinsic_data_[frame_id] = data;
  return true;
}

bool CalibrationData::get_camera_intrinsic_data(
  const std::string & frame_id, std::string & camera_model_type, std::vector<double> & intrinsics,
  std::vector<double> & distortion_coeffs)
{
  auto it = camera_intrinsic_data_.find(frame_id);
  if (it == camera_intrinsic_data_.end()) {
    return false;
  }
  camera_model_type = it->second.camera_model_type;
  intrinsics = it->second.intrinsics;
  distortion_coeffs = it->second.distortion_coeffs;
  return true;
}

void CalibrationData::remove_camera_intrinsic_data(const std::string & frame_id)
{
  auto it = camera_intrinsic_data_.find(frame_id);
  if (it != camera_intrinsic_data_.end()) {
    camera_intrinsic_data_.erase(it);
  }
}

bool CalibrationData::add_extrinsic_data(
  const std::string & frame_id, const std::string & child_frame_id,
  const Eigen::Matrix4d & transform)
{
  auto pair = CalibrationData::SensorFrameIdPair(frame_id, child_frame_id);
  extrinsic_data_[pair] = transform.eval();
  return true;
}

bool CalibrationData::get_extrinsic_data(
  const std::string & frame_id, const std::string & child_frame_id, Eigen::Matrix4d & transform)
{
  auto pair = CalibrationData::SensorFrameIdPair(frame_id, child_frame_id);
  auto it = extrinsic_data_.find(pair);
  if (it == extrinsic_data_.end()) {
    return false;
  }
  transform = it->second;
  return true;
}

void CalibrationData::remove_extrinsic_data(
  const std::string & frame_id, const std::string & child_frame_id)
{
  auto pair = CalibrationData::SensorFrameIdPair(frame_id, child_frame_id);
  auto it = extrinsic_data_.find(pair);
  if (it != extrinsic_data_.end()) {
    extrinsic_data_.erase(it);
  }
}

bool CalibrationData::save(const std::string & file)
{
  std::ofstream ofs;
  ofs.open(file);
  if (!ofs) {
    return false;
  }
  ofs.setf(std::ios::fixed, std::ios::floatfield);
  ofs.precision(6);
  if (!camera_intrinsic_data_.empty()) {
    ofs << "cameras:" << std::endl;
    int cnt = 1;
    for (auto & [frame_id, data] : camera_intrinsic_data_) {
      ofs << "    camera" << cnt << ":" << std::endl;
      ofs << "        frame_id: " << frame_id << std::endl;
      ofs << "        camera_model_type: " << data.camera_model_type << std::endl;
      ofs << "        intrinsics: " << to_string(data.intrinsics) << std::endl;
      ofs << "        distortion_coeffs: " << to_string(data.distortion_coeffs) << std::endl;
      cnt++;
    }
  }
  if (!extrinsic_data_.empty()) {
    ofs << "sensor_pair_transforms:" << std::endl;
    int cnt = 1;
    for (auto & [frame_id_pair, transform] : extrinsic_data_) {
      auto t = transform.block<3, 1>(0, 3);
      std::vector<double> translation{t.x(), t.y(), t.z()};
      auto q = Eigen::Quaterniond(transform.block<3, 3>(0, 0));
      std::vector<double> rotation{q.x(), q.y(), q.z(), q.w()};
      ofs << "    transform" << cnt << ":" << std::endl;
      ofs << "        frame_id: " << frame_id_pair.first << std::endl;
      ofs << "        child_frame_id: " << frame_id_pair.second << std::endl;
      ofs << "        translation: " << to_string(translation) << std::endl;
      ofs << "        rotation: " << to_string(rotation) << std::endl;
      cnt++;
    }
  }
  ofs << std::endl;
  return true;
}

bool CalibrationData::load(const std::string & file)
{
  camera_intrinsic_data_.clear();
  extrinsic_data_.clear();
  YAML::Node config = YAML::LoadFile(file);
  auto cameras = config["cameras"];
  if (cameras.IsDefined() && cameras.IsMap()) {
    for (auto it = cameras.begin(); it != cameras.end(); ++it) {
      std::string key = "unknown";
      try {
        key = it->first.as<std::string>();
        YAML::Node camera = it->second;
        auto frame_id = camera["frame_id"].as<std::string>();
        auto camera_model_type = camera["camera_model_type"].as<std::string>();
        auto intrinsics = camera["intrinsics"].as<std::vector<double>>();
        auto distortion_coeffs = camera["distortion_coeffs"].as<std::vector<double>>();
        add_camera_intrinsic_data(frame_id, camera_model_type, intrinsics, distortion_coeffs);
      } catch (std::exception & e) {
        error_message_ = std::string("invalid camera data [") + key + "]";
        return false;
      }
    }
  }
  auto sensor_pair_transforms = config["sensor_pair_transforms"];
  if (sensor_pair_transforms.IsDefined() && sensor_pair_transforms.IsMap()) {
    for (auto it = sensor_pair_transforms.begin(); it != sensor_pair_transforms.end(); ++it) {
      std::string key = "unknown";
      try {
        key = it->first.as<std::string>();
        YAML::Node sensor_pair_transform = it->second;
        auto frame_id = sensor_pair_transform["frame_id"].as<std::string>();
        auto child_frame_id = sensor_pair_transform["child_frame_id"].as<std::string>();
        auto t = sensor_pair_transform["translation"].as<std::vector<double>>();
        auto r = sensor_pair_transform["rotation"].as<std::vector<double>>();
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 1>(0, 3) = Eigen::Vector3d(t[0], t[1], t[2]);
        transform.block<3, 3>(0, 0) = Eigen::Quaterniond(r[3], r[0], r[1], r[2]).toRotationMatrix();
        add_extrinsic_data(frame_id, child_frame_id, transform);
      } catch (std::exception & e) {
        error_message_ = std::string("invalid sensor_pair_transform [") + key + "]";
        return false;
      }
    }
  }
  return true;
}

std::string CalibrationData::error_message()
{
  return error_message_;
}

}  // namespace calibration_common
