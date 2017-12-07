// Author: Felix Engström
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <random>

static const float PI = acos(-1);

// Help function to get angle from quaternion msg
float quat2angle(geometry_msgs::Quaternion q){
    tf::Quaternion q2;
    tf::quaternionMsgToTF(q, q2);
    return tf::getYaw(q2);
}
    
// Error detector listenes to est posision and compares the change in estimated posision 
// (from the localisation) with the change in odometry. If it is to big it publishes on an 
// error topic.

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
        double xDiffEst;
        double yDiffEst;
        double omegaDiffEst;
        double totDiffEst;

        double xDiffOdom;
        double yDiffOdom;
        double omegaDiffOdom;
        double totDiffOdom;
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

        //calculate the change in estimated position
        xDiffEst = msg->pose.position.x - lastEstPose.pose.position.x;
        yDiffEst = msg->pose.position.y - lastEstPose.pose.position.y;
        omegaDiffEst = quat2angle(msg->pose.orientation)-quat2angle(lastEstPose.pose.orientation);
        totDiffEst = std::sqrt(std::pow(xDiffEst,2) + std::pow(yDiffEst, 2));

        //calculate the change in odometry
        xDiffOdom = transform.getOrigin().x();
        yDiffOdom = transform.getOrigin().y();
        omegaDiffOdom = tf::getYaw(transform.getRotation());
        totDiffOdom = std::sqrt(std::pow(xDiffOdom,2) + std::pow(yDiffOdom, 2));
        
        // Compare the differences
        if (std::abs(totDiffEst-totDiffOdom) > 0.08 || std::abs(omegaDiffEst-omegaDiffOdom) > 0.11)
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

