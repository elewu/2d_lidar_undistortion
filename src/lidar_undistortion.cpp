#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <cmath>

#include <boost/circular_buffer.hpp>


#include <chrono>

using namespace std;
using namespace chrono;

typedef boost::circular_buffer<sensor_msgs::Imu> ImuCircularBuffer;

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator()
    {
      ros::NodeHandle nh_param("~");

      nh_param.param<double>("imu_frequency", xsens_frequency, 100.0);
      nh_param.param<double>("msg_delay_time", msg_delay_time, 10.0);
      nh_param.param<bool>("display_raw_scan", display_raw_scan, false);

      lidar_sub_ = nh_.subscribe("/scan", 10, &LidarMotionCalibrator::LidarCallback, this);
      imu_sub_ = nh_.subscribe("/mti/sensor/imu", 100, &LidarMotionCalibrator::ImuCallback, this);

      pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_output/after", 10);
      pcl_pub_origin_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_output/origin", 10);
      pcl_pub_test_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_output/test", 10);

      delay_duration = ros::Duration(msg_delay_time / 1000.0);

      //stores 200 frames of imu data(1s)
      imuCircularBuffer = ImuCircularBuffer(100);
    }


    ~LidarMotionCalibrator()
    {

    }

    void ImuCallback(const sensor_msgs::ImuConstPtr& _imu_msg)
    {
      imuCircularBuffer.push_front(*_imu_msg);
      cout << "buffer size: " << imuCircularBuffer.size() << endl;
    }

    void LidarCallback(const sensor_msgs::LaserScanConstPtr& _lidar_msg)
    {

      cout << "lidar message received" << endl;

      //激光点的个数
      int length = _lidar_msg->ranges.size();
      cout << "point size: " << length << endl;


      //当前激光帧从头到尾的时间
      frame_duration = ros::Duration(_lidar_msg->scan_time);

      //估计第一个激光点的时刻
      ros::Time lidar_timebase = ros::Time::now() - frame_duration - delay_duration;

      pcl::PointCloud<pcl::PointXYZI> pointcloud_raw_pcl;
      pcl::PointCloud<pcl::PointXYZI> pointcloud_test_pcl;

      LaserScanToPointCloud(*_lidar_msg, pointcloud_raw_pcl);

      //保证imu buffer中有足够多的数据
      if (imuCircularBuffer.size() > frame_duration.toSec() * xsens_frequency * 1.5)
      {

        auto start = system_clock::now();

        sensor_msgs::PointCloud2 pointcloud_msg;
        sensor_msgs::PointCloud2 pointcloud_raw_msg;
        pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
        pcl::PointXYZI point_xyzi;

        geometry_msgs::Quaternion current_orientation = imuCircularBuffer[0].orientation;
        Eigen::Quaternionf current_quat(current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z);

        for (int i = 0; i < length; i++)
        {
          pcl::PointXYZI current_point = pointcloud_raw_pcl[i];

          Eigen::Vector3f point_in(current_point.x, current_point.y, 0.0);

          ros::Time point_timestamp = lidar_timebase + ros::Duration(_lidar_msg->time_increment * i);

          Eigen::Quaternionf point_quat;

          if (getLaserPose(i, point_timestamp, point_quat) == true)
          {
            Eigen::Quaternionf delta_quat = current_quat.inverse() * point_quat;

            Eigen::Vector3f point_out = delta_quat * point_in;

            point_xyzi.x = point_out(0);
            point_xyzi.y = point_out(1);
            point_xyzi.z = 0.0;
            point_xyzi.intensity = 1.0;
            pointcloud_pcl.push_back(point_xyzi);

          }
          else
          {
            break;
          }
        }
        pcl::toROSMsg(pointcloud_pcl, pointcloud_msg);
        pointcloud_msg.header.frame_id = "laser";
        pcl_pub_.publish(pointcloud_msg);

        if (display_raw_scan == true)
        {
          pcl::toROSMsg(pointcloud_raw_pcl, pointcloud_raw_msg);
          pointcloud_raw_msg.header.frame_id = "laser";
          pcl_pub_origin_.publish(pointcloud_raw_msg);

          for (int i = 0; i < 20; i++)
          {
            pointcloud_test_pcl.push_back(pointcloud_raw_pcl[719 - i]);
          }
          pcl::toROSMsg(pointcloud_test_pcl, pointcloud_raw_msg);
          pointcloud_raw_msg.header.frame_id = "laser";
          pcl_pub_test_.publish(pointcloud_raw_msg);
        }

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        cout <<  "spend "  << double(duration.count()) / 1000.0 << " miliseconds" << endl;
      }
      else
      {
        cout << "imu data less than 0.2s, waiting for next packet" << endl;
      }
    }

    Eigen::Matrix4d QuaternionToMatrix4d(geometry_msgs::Quaternion quat)
    {
      double x = quat.x;
      double y = quat.y;
      double z = quat.z;
      double w = quat.w;

      Eigen::Matrix4d transformT;
      transformT << 1.0 - 2.0*y*y - 2.0*z*z, 2.0*x*y - 2.0*z*w      , 2.0*x*z + 2.0*y*w       , 0.0,
                    2.0*x*y + 2.0*z*w      , 1.0 - 2.0*x*x - 2.0*z*z, 2.0*y*z - 2.0*x*w       , 0.0,
                    2.0*x*z - 2.0*y*w      , 2.0*y*z + 2.0*x*w      , 1.0 - 2.0f*x*x - 2.0*y*y, 0.0,
                    0.0                    , 0.0                    , 0.0                     , 1.0;
      return transformT;
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在当前激光点的姿态，返回是否成功
     * @param point_index  激光点的序号
     * @param _timestamp   当前激光点的时间戳
     * @param quat_out  输出四元数
    */
    bool getLaserPose(int point_index, ros::Time &_timestamp, Eigen::Quaternionf &quat_out)
    {
      static int index_front = 0;
      static int index_back = 0;
      static ros::Time timestamp_front;
      static ros::Time timestamp_back;
      static Eigen::Quaternionf quat_front, quat_back;

      static bool index_updated_flag = false;

      if (point_index == 0)
      {
        int i = 0;
        while (_timestamp < imuCircularBuffer[i].header.stamp)
        {
          i++;
        }
        index_front = i - 1;
        index_back = i;
        index_updated_flag = true;

      }
      else
      {
        while (_timestamp > imuCircularBuffer[index_front].header.stamp
               && _timestamp > imuCircularBuffer[index_back].header.stamp)
        {
          index_front--;
          index_back--;

          if (index_front < 0)
          {
            cout << "loss imu data" << endl;
            return false;
          }

          index_updated_flag = true;
        }
      }

      if (index_updated_flag == true)
      {
        cout << "index updated: " << index_front << " " << index_back << endl;
        timestamp_front = imuCircularBuffer[index_front].header.stamp;
        timestamp_back = imuCircularBuffer[index_back].header.stamp;
        quat_front = Eigen::Quaternionf(imuCircularBuffer[index_front].orientation.w, imuCircularBuffer[index_front].orientation.x, imuCircularBuffer[index_front].orientation.y, imuCircularBuffer[index_front].orientation.z);
        quat_back = Eigen::Quaternionf(imuCircularBuffer[index_back].orientation.w, imuCircularBuffer[index_back].orientation.x, imuCircularBuffer[index_back].orientation.y, imuCircularBuffer[index_back].orientation.z);

        index_updated_flag = false;
      }

      float alpha = (float)(_timestamp.toNSec() - timestamp_back.toNSec()) / (timestamp_front.toNSec() - timestamp_back.toNSec());
      if (point_index > 700)
      {
        cout << "alpha:  " << alpha << endl;
      }
      //球面线性插值
      quat_out = quat_back.slerp(alpha, quat_front);

      return true;
    }

    void LaserScanToPointCloud(sensor_msgs::LaserScan _laser_scan, pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
    {
      _pointcloud.clear();
      pcl::PointXYZI newPoint;
      newPoint.z = 0.0;
      newPoint.intensity = 1.0;
      double newPointAngle;

      int beamNum = _laser_scan.ranges.size();
      for (int i = 11; i >= 0; i--)
      {
        newPointAngle = _laser_scan.angle_min + _laser_scan.angle_increment * i;
        newPoint.x = _laser_scan.ranges[i] * cos(newPointAngle);
        newPoint.y = _laser_scan.ranges[i] * sin(newPointAngle);
        _pointcloud.push_back(newPoint);
      }
      for (int i = beamNum - 1; i >= 17; i--)
      {
        newPointAngle = _laser_scan.angle_min + _laser_scan.angle_increment * i;
        newPoint.x = _laser_scan.ranges[i] * cos(newPointAngle);
        newPoint.y = _laser_scan.ranges[i] * sin(newPointAngle);
        _pointcloud.push_back(newPoint);
      }
    }


public:

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher pcl_pub_;
    ros::Publisher pcl_pub_origin_;
    ros::Publisher pcl_pub_test_;

    ros::Duration frame_duration, delay_duration;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;

    ImuCircularBuffer imuCircularBuffer;

    double xsens_frequency, msg_delay_time;
    bool display_raw_scan;

};


int main(int argc,char ** argv)
{
    ros::init(argc, argv, "lidar_undistortion");

    LidarMotionCalibrator tmpLidarMotionCalib;

    ros::spin();

    return 0;
}


