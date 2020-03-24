# 2d_lidar_undistortion
A ros package used to correct motion distortion of a 2D LIDAR.

## Introduction
利用IMU数据对2D激光雷达数据进行运动畸变校正（仅限于旋转校正）。imu的更新频率高于100Hz时，效果更佳。

This ros package uses standard IMU message to correct motion distortion of a 2D LIDAR in real time(rotation only). It is better that the update frequency of IMU is higher than 100Hz.

## Usage
订阅话题：**sensor_msgs::LaserScan** 以及 **sensor_msgs::Imu**

Subcribed topics: **sensor_msgs::LaserScan** and **sensor_msgs::Imu**
