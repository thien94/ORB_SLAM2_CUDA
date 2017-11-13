
#ifndef SLAMDATA_H
#define SLAMDATA_H


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>

//#include <pcl/visualization/cloud_viewer.h> 
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h> 
// #include <pcl_ros/transforms.h>
// #include <pcl/point_cloud.h>  
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>

#include "../../../include/System.h"
#include "../../../include/Converter.h"

#define MAP_SCALE 1.0f

enum TimePointIndex {
    TIME_BEGIN,
    TIME_FINISH_CV_PROCESS,
    TIME_FINISH_SLAM_PROCESS
};

void SaveTimePoint(TimePointIndex index, std::chrono::steady_clock::time_point time_point);

void CalculateAndPrintOutProcessingFrequency(void);

void PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr);

void PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr);

void PerformTFTransformAndPublish(ros::NodeHandle *nodeHandler);



#endif // SLAMDATA_H

