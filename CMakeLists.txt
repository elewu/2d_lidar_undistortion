cmake_minimum_required(VERSION 2.8.3)
project(lidar_undistortion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LaserUndistortion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(lidar_undistortion src/lidar_undistortion.cpp)

target_link_libraries(lidar_undistortion
  ${catkin_LIBRARIES}
)



