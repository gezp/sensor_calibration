# sensor_calibration

该项目基于ROS2平台实现了部分传感器标定功能，主要以学习为目的，面向初学者学习入门，因此代码侧重于可读性，及扩展性，尽可能将代码进行解耦，并遵循ROS2的项目规范，以及代码风格。

本项目主要面向以下场景的传感器标定：

* 传感器内参标定（sensor_intrinsics）
* 传感器到传感器之间的外参标定（sensor2sensor）

## Quick start

环境要求

- ROS版本: `Humble`

下载源码及安装依赖

```
# 使用git下载代码到ROS2工作空间的src目录
# mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/gezp/sensor_calibration.git
# 进入ROS2工作空间, 安装依赖
# cd  ~/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
```

编译

```
colcon build --symlink-install
```

## Plan

传感器内参标定

- [x] 实现针孔相机内参标定: `camera_intrinsic_calibration`
- [ ] 实现鱼眼相机内参标定: `camera_intrinsic_calibration`
- [ ] 实现全景相机内参标定: `camera_intrinsic_calibration`

sensor2sensor外参标定

- [ ] 实现手动lidar2lidar外参标定: `lidar_lidar_manual_calibration`
- [ ] 实现基于ICP的自动lidar2lidar外参标定: `lidar_lidar_calibration`
- [ ] 实现手动lidar2camera外参标定: `lidar_camera_manual_calibration`


## Acknowledgement

本项目参考了许多其它类似项目，并参考并引用了部分代码，这里向以下项目的作者表示感谢！

- https://github.com/PJLab-ADG/SensorsCalibration
- https://github.com/tier4/CalibrationTools
