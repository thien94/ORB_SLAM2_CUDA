
#ifndef SLAMDATA_H
#define SLAMDATA_H

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <mutex>

#define MAP_SCALE 1.0f

enum TimePointIndex {
    TIME_BEGIN,
    TIME_FINISH_CV_PROCESS,
    TIME_FINISH_SLAM_PROCESS
};

void SaveTimePoint(TimePointIndex index, std::chrono::steady_clock::time_point time_point);

void CalculateAndOutputProcessingFrequency(void);

void PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr);

void PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr);

void PerformTFTransformAndPublish(void);



#endif // SLAMDATA_H

