# Copyright 2024 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_calibration_common = get_package_share_directory("calibration_common")
    data_dir = os.path.join(
        os.environ["HOME"], "calibration_data", "SensorsCalibration"
    )
    lidar_data_dir = os.path.join(data_dir, "lidar2camera", "lidar")
    camera_data_dir = os.path.join(data_dir, "camera_intrinsic")
    dummy_lidar_node = Node(
        name="dummy_lidar_node",
        package="calibration_common",
        executable="dummy_lidar_node",
        parameters=[{"data_dir": lidar_data_dir, "frame_id": "top_center_lidar"}],
        output="screen",
    )
    dummy_camera_node = Node(
        name="dummy_camera_node",
        package="calibration_common",
        executable="dummy_camera_node",
        parameters=[
            {
                "data_dir": camera_data_dir,
                "frame_id": "center_camera",
                "rate": 1.0,
            }
        ],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(dummy_lidar_node)
    ld.add_action(dummy_camera_node)
    return ld
