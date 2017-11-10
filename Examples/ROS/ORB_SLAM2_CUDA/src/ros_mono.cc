/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<tf/transform_broadcaster.h>

#include<ros/ros.h>
#include "../../../include/Converter.h"
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

ros::Publisher pose_pub;
ros::Publisher pose_inc_pub;
tf::Transform last_transform;
int frame_num = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    bool bUseViewer;

    if(argc == 3)
    {
        ROS_WARN_STREAM("bUseViewer not set, use default value");
        bUseViewer = false;
    }
    else if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings bUseViewer" << endl;        
        ros::shutdown();
        return 1;
    }    
    
    stringstream ss(argv[3]);
    ss >> boolalpha >> bUseViewer;
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, bUseViewer);
    
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    // Perform tf transform and publish
    last_transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    last_transform.setRotation(q);

    pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("posestamped", 1000);
    pose_inc_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("incremental_pose_cov", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/ubuntu/ORB_SLAM2_CUDA/test_results/Mono_KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void CalculateAndOutputProcessingFrequency(std::chrono::steady_clock::time_point t1, std::chrono::steady_clock::time_point t2, std::chrono::steady_clock::time_point t3)
{
    static long spinCnt = 0;
    static double t_temp = 0;
    double time_read= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    double time_track= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
    double time_total= std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
    
    cout << "Image reading time = " << setw(10) << time_read  << "s" << endl;
    cout << "Tracking time =      " << setw(10) << time_track << "s, frequency = " << 1/time_track << "Hz" << endl; 
    cout << "ALL cost time =      " << setw(10) << time_total << "s, frequency = " << 1/time_total << "Hz" << endl; 
    t_temp = (time_total + t_temp*spinCnt)/(1+spinCnt);
    cout << "Avg. time =          " << setw(10) << t_temp     << "s, frequency = " << 1/t_temp     << "Hz" << endl;
    cout << "\n\n" << endl;
    spinCnt++;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Time point 1: Begin processing 1 frame
    std::chrono::steady_clock::time_point tp1 = std::chrono::steady_clock::now();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Time point 2: Finish processing 1 frame
    std::chrono::steady_clock::time_point tp2 = std::chrono::steady_clock::now();

    /**********************************************
      tf publisher
    **********************************************/
    // mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    // Time point 3: Finish SLAM process for this frame
    std::chrono::steady_clock::time_point tp3 = std::chrono::steady_clock::now();

    CalculateAndOutputProcessingFrequency(tp1, tp2, tp3);

    if (Tcw.empty()) {
      return;
    }

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    const float MAP_SCALE = 1.0f;

    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));
    tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
    transform.setRotation(tf_quaternion);

    br.sendTransform(tf::StampedTransform(transform, ros::Time(cv_ptr->header.stamp.toSec()), "world", "ORB_SLAM2"));


    /**********************************************
      pose publisher
    **********************************************/

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cv_ptr->header.stamp;
    pose.header.frame_id ="world";
    tf::poseTFToMsg(transform, pose.pose);
    pose_pub.publish(pose);
    geometry_msgs::PoseWithCovarianceStamped pose_inc_cov;
    pose_inc_cov.header.stamp = cv_ptr->header.stamp;
    pose_inc_cov.header.frame_id = "keyframe_" + to_string(frame_num++);
    tf::poseTFToMsg(last_transform.inverse()*transform, pose_inc_cov.pose.pose);
    pose_inc_cov.pose.covariance[0*7] = 0.0005;
    pose_inc_cov.pose.covariance[1*7] = 0.0005;
    pose_inc_cov.pose.covariance[2*7] = 0.0005;
    pose_inc_cov.pose.covariance[3*7] = 0.0001;
    pose_inc_cov.pose.covariance[4*7] = 0.0001;
    pose_inc_cov.pose.covariance[5*7] = 0.0001;

    pose_inc_pub.publish(pose_inc_cov);

    last_transform = transform;



}


