#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <random>
//#include "sensor_msgs/PointCloud.h"
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include "geometry_msgs/PoseArray.h"
//#include "geometry_msgs/Point.h"
//#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/PoseStamped.h"
static const float PI = acos(-1);

float quat2angle(geometry_msgs::Quaternion q){
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    return tf::getYaw(q2);
}
    
class ErrorDetector{

    private:
        bool hasReading;
        geometry_msgs::PoseStamped lastEstPose;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        tf::TransformListener listener;

    public:
        ErrorDetector(ros::NodeHandle n_);
        void estPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

};

ErrorDetector::ErrorDetector(ros::NodeHandle n_):n(n_),listener(), hasReading(false)
{
    n = n_;
    ROS_INFO("starting node");
    sub = n.subscribe("/localisation/pose", 1, &ErrorDetector::estPoseCallback, this);
    pub = n.advertise<std_msgs::Bool>("/robot/odomdiff", 1);
}
void ErrorDetector::estPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (hasReading)
    {
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("odom",lastEstPose.header.stamp, "odom",
                                  msg->header.stamp,"map", ros::Duration(2));
            listener.lookupTransform("odom",lastEstPose.header.stamp, "odom",
                                  msg->header.stamp,"map", transform);
        } catch(tf::TransformException &ex)
        {
            ROS_INFO("odom transform not found in not found in error detection node");
            return;
        }
        double xDiffEst = msg->pose.position.x - lastEstPose.pose.position.x;
        double yDiffEst = msg->pose.position.y - lastEstPose.pose.position.y;
        double omegaDiffEst = quat2angle(msg->pose.orientation)-quat2angle(lastEstPose.pose.orientation);
        double totDiffEst = std::sqrt(std::pow(xDiffEst,2) + std::pow(yDiffEst, 2));

        double xDiffOdom = transform.getOrigin().x();
        double yDiffOdom = transform.getOrigin().y();
        double omegaDiffOdom = tf::getYaw(transform.getRotation());
        double totDiffOdom = std::sqrt(std::pow(xDiffOdom,2) + std::pow(yDiffOdom, 2));
        if (std::abs(totDiffEst-totDiffOdom) > 0.1 || std::abs(omegaDiffEst-omegaDiffOdom) > 0.2)
        {
            std_msgs::Bool b;
            b.data = true;
            pub.publish(b);
        }
    }
    lastEstPose = geometry_msgs::PoseStamped(*msg);
    hasReading = true;
}
int main(int argc, char*argv[])
{
    ros::init(argc, argv, "error_detector");
    ros::NodeHandle n;
    ErrorDetector ed(n);
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

