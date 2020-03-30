# 2d_lidar_undistortion
A ros package used to correct motion distortion of a 2D LIDAR.

## Introduction
利用IMU数据对2D激光雷达数据进行运动畸变校正（仅限于旋转校正）。imu的更新频率高于100Hz时，效果更佳。

This ros package uses standard IMU message to correct motion distortion of a 2D LIDAR in real time(rotation only). It is better that the update frequency of IMU is higher than 100Hz.

## Usage
订阅话题：`sensor_msgs::LaserScan` 以及 `sensor_msgs::Imu`

Subcribed topics: `sensor_msgs::LaserScan` and `sensor_msgs::Imu`

A rosbag for test [2d_undistortion_example_1.bag](https://drive.google.com/file/d/1aic5VevWkU7q3yLxUEGD6XcEBUR9QVRZ/view?usp=sharing). The rosbag is recorded with [RPLIDAR A1](http://www.slamtec.com/en/Lidar/A1) and Xsens MTI-300.
#### Testing
1. Start ros master.
`roscore`
2. Play the rosbag with sim time and X0.5 speed, and press spacebar to pause. 
`rosbag play 2d_undistortion_example_1.bag --clock -r 0.5` 
3. Start lidar_undistortion node.
`roslaunch lidar_undistortion lidar_undistortion_offline.launch`
4. Press spacebar in the rosbag terminal to continue playing data.
5. The results will be shown in rviz. Raw scan is displayed with red points and output is displayed with white points.
