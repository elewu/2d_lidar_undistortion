# 2d_lidar_undistortion
A ros package used to correct motion distortion of a 2D LIDAR.

## Introduction
利用IMU数据对2D激光雷达数据进行运动畸变校正（仅限于旋转校正）。imu的更新频率高于100Hz时，效果更佳。

This ros package uses standard IMU message to correct motion distortion of a 2D LIDAR in real time(rotation only). It is better that the update frequency of IMU is higher than 100Hz.

## Usage
订阅话题：`sensor_msgs::LaserScan` 以及 `sensor_msgs::Imu`

Subcribed topics: `sensor_msgs::LaserScan` and `sensor_msgs::Imu`

A rosbag for test [2d_undistortion_example_1.bag](https://drive.google.com/file/d/1aic5VevWkU7q3yLxUEGD6XcEBUR9QVRZ/view?usp=sharing). The rosbag is recorded with [RPLIDAR A1](http://www.slamtec.com/en/Lidar/A1) and Xsens MTI-300. (百度云链接[2d_undistortion_example_1.bag](https://pan.baidu.com/s/19Oqvab1HYZShCINIwoQ1Eg)，提取码`r00m`）

## How to test the rosbag?
1. Start ros master.
`roscore`
2. Play the rosbag with sim time and X0.5 speed, and press spacebar to pause. 
`rosbag play 2d_undistortion_example_1.bag --clock -r 0.5` 
3. Start lidar_undistortion node.
`roslaunch lidar_undistortion lidar_undistortion_offline.launch`
4. Press spacebar in the rosbag terminal to continue playing data.
5. The results will be shown in rviz. Raw scan is displayed with red points and output is displayed with white points.
![result](https://github.com/elewu/2d_lidar_undistortion/blob/master/pics/result.png)

## Parameters in launch file
| 参数名 | 类型 | 解释 |
|--|--|--|
| lidar_topic | string| 订阅的激光数据话题名|
| lidar_msg_delay_time | double | 雷达数据传输滞后的估计，单位毫秒，一般在10ms内 |
|scan_direction_clockwise|bool|激光雷达扫描旋转方向，true为顺时针，false为逆时针；方向不对会增大畸变|
| imu_topic | string | 订阅的imu话题名 |
| imu_frequency | double | 估计的imu发布频率，仅用来确定队列长度，不需要很精确 |
| output_frame_id | string | 输出消息的frame_id |  
| pub_raw_scan_pointcloud | bool | 是否发布校正前的点云 |  
| pub_laserscan | bool | 是否将校正后的数据重新封装为LaserScan消息发布 |  
| laserscan_angle_increment | double | 接上条，发布的LaserScan消息的角度分辨率，分辨率越小，转换精度越高，单位为弧度（空白数据用nan填充） |  
| use_range_filter | bool | 是否限定输出的扫描距离 |  
| range_filter_min | double | 扫描距离的最小值 |  
| range_filter_max | double | 扫描距离的最大值 |  
| use_angle_filter | bool | 是否限定输出的角度范围，默认为$-\pi$到$+\pi$ |  
| angle_filter_min | double | 扫描角度的最小值，单位为弧度 |  
| angle_filter_max | double | 扫描角度的最大值，单位为弧度 |  
| use_radius_outlier_filter | bool | 是否使用PCL库的RadiusOutlierRemoval滤波器，用来移除离群点 |  
| radius_outlier_filter_search_radius | double | RadiusOutlierRemoval滤波器参数，搜索半径 |  
| radius_outlier_filter_min_neighbors | double | RadiusOutlierRemoval滤波器参数，有效点的最少近邻点数量 |  
