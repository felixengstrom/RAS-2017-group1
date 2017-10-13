#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

ros::Publisher motor_pub;
geometry_msgs::Pose2D pose;
int angle_ok = 0;
int distance_ok = 0;

void teleopCallback (const geometry_msgs::Pose2D::ConstPtr& msg) {
   pose = *msg;
   angle_ok = 0;
   distance_ok = 0;
}

void odometryCallback (const geometry_msgs::Pose2D::ConstPtr& msg) {
   geometry_msgs::Pose2D robot_pose = *msg;
   geometry_msgs::Twist move;
   double theta = atan2(pose.y-robot_pose.y, pose.x-robot_pose.x);
   ROS_INFO("Angle desired: %f", theta-robot_pose.theta);
   double distance = std::sqrt(std::pow(pose.x-robot_pose.x,2)+std::pow(pose.y-robot_pose.y,2));
   if (std::abs(theta-robot_pose.theta) < 0.18 || distance <= 0.03) {
	if (distance > 0.03) {
	    if (distance > 0.1)
	    	move.linear.x = 0.02;
	    else
		move.linear.x = distance/10;
	    move.angular.z = 0;
	    motor_pub.publish(move);
	    ROS_INFO("%f",distance);
	}
	else {
	    move.linear.x = 0;
	    move.angular.z = 0;
	    motor_pub.publish(move);
	}
   }
   else {
	move.linear.x = 0;
	if (std::abs(theta-robot_pose.theta) > 0.20) {
	    if (theta-robot_pose.theta > 0)
	    	move.angular.z = 0.1;
	    else
		move.angular.z = -0.1;
	}
	else {
	    if (theta-robot_pose.theta > 0)
	    	move.angular.z = 0.1;
	    else
		move.angular.z = -0.1;
	}
	double distance = std::sqrt(std::pow(pose.x-robot_pose.x, 2) + std::pow(pose.y-robot_pose.y, 2));
	ROS_INFO("%f ; %f", std::abs(theta-robot_pose.theta), distance);
	motor_pub.publish(move);
   }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_following");
    ros::NodeHandle n;
    ros::Subscriber teleop_sub = n.subscribe("/pose_teleop", 10, teleopCallback);
    ros::Subscriber odometry_sub = n.subscribe("/robot/pose", 10, odometryCallback);
    motor_pub = n.advertise<geometry_msgs::Twist>("motor_teleop/twist", 10);
    ros::Rate loop_rate(10);
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;
    while(ros::ok()) {
	ros::spin();
    }
}
