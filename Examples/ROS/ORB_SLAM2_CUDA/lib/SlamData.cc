#include "SlamData.h"

using namespace std;

ros::Publisher pose_pub;

ros::Publisher pose_inc_pub;

std::chrono::steady_clock::time_point tp1, tp2, tp3;

tf::Transform new_transform, last_transform;

void SaveTimePoint(TimePointIndex index, std::chrono::steady_clock::time_point time_point)
{
    switch (index)
    {
	case TIME_BEGIN:
    	tp1 = time_point;
		break;
	case TIME_FINISH_CV_PROCESS:
    	tp2 = time_point;
		break;
	case TIME_FINISH_SLAM_PROCESS:
    	tp3 = time_point;
        break;
    default: 
        break;
    }
}

void CalculateAndPrintOutProcessingFrequency(void)
{
    static long spinCnt = 0;
    static double t_temp = 0;

    double time_read= std::chrono::duration_cast<std::chrono::duration<double> >(tp2 - tp1).count();
    double time_track= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp2).count();
    double time_total= std::chrono::duration_cast<std::chrono::duration<double> >(tp3 - tp1).count();
    
    cout << "Image reading time = " << setw(10) << time_read  << "s" << endl;
    cout << "Tracking time =      " << setw(10) << time_track << "s, frequency = " << 1/time_track << "Hz" << endl; 
    cout << "ALL cost time =      " << setw(10) << time_total << "s, frequency = " << 1/time_total << "Hz" << endl; 
    t_temp = (time_total + t_temp*spinCnt)/(1+spinCnt);
    cout << "Avg. time =          " << setw(10) << t_temp     << "s, frequency = " << 1/t_temp     << "Hz" << endl;
    cout << "\n\n" << endl;

    spinCnt++;
}

void PublishTFForROS(cv::Mat Tcw, cv_bridge::CvImageConstPtr cv_ptr)
{
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    static tf::TransformBroadcaster br;

    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0) * MAP_SCALE, twc.at<float>(0, 1) * MAP_SCALE, twc.at<float>(0, 2) * MAP_SCALE));

    tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);

    new_transform.setRotation(tf_quaternion);

    br.sendTransform(tf::StampedTransform(new_transform, ros::Time(cv_ptr->header.stamp.toSec()), "world", "ORB_SLAM2"));
}



void PublishPoseForROS(cv_bridge::CvImageConstPtr cv_ptr)
{
    static int frame_num = 0;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = cv_ptr->header.stamp;
    pose.header.frame_id ="world";
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);
    geometry_msgs::PoseWithCovarianceStamped pose_inc_cov;
    pose_inc_cov.header.stamp = cv_ptr->header.stamp;
    pose_inc_cov.header.frame_id = "keyframe_" + to_string(frame_num++);
    tf::poseTFToMsg(last_transform.inverse()*new_transform, pose_inc_cov.pose.pose);
    pose_inc_cov.pose.covariance[0*7] = 0.0005;
    pose_inc_cov.pose.covariance[1*7] = 0.0005;
    pose_inc_cov.pose.covariance[2*7] = 0.0005;
    pose_inc_cov.pose.covariance[3*7] = 0.0001;
    pose_inc_cov.pose.covariance[4*7] = 0.0001;
    pose_inc_cov.pose.covariance[5*7] = 0.0001;

    pose_inc_pub.publish(pose_inc_cov);

    last_transform = new_transform;

}

void PerformTFTransformAndPublish(ros::NodeHandle *nodeHandler)
{
    // Perform tf transform and publish
    last_transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    last_transform.setRotation(q);

    pose_pub = (*nodeHandler).advertise<geometry_msgs::PoseStamped>("posestamped", 1000);
    pose_inc_pub = (*nodeHandler).advertise<geometry_msgs::PoseWithCovarianceStamped>("incremental_pose_cov", 1000);
}
