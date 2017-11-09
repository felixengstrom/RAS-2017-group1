#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

ros::Publisher motor_pub;
geometry_msgs::PoseArray poses;
int current_pose;
int nb_poses;
int angle_ok = 0;
int distance_ok = 0;

void teleopCallback (const geometry_msgs::PoseArray::ConstPtr& msg) {
   poses = *msg;
   current_pose = 0;
   nb_poses = poses.poses.size();
}

void odometryCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
   geometry_msgs::PoseStamped robot_pose = *msg;
   geometry_msgs::Twist move;
   double theta = atan2(poses.poses[current_pose].position.y-robot_pose.pose.position.y, poses.poses[current_pose].position.x-robot_pose.pose.position.x);
   float desired_angle = theta-(atan2(robot_pose.pose.orientation.z,robot_pose.pose.orientation.w)*2.0);
   ROS_INFO("Angle desired: %f", desired_angle);
   double distance = std::sqrt(std::pow(poses.poses[current_pose].position.x-robot_pose.pose.position.x,2)+std::pow(poses.poses[current_pose].position.y-robot_pose.pose.position.y,2));
   if (std::abs(desired_angle) < 0.05 || distance <= 0.01) {
	if (distance > 0.01) {
//	    if (distance > 0.1)
	    	move.linear.x = 0.1;
//	    else
//		move.linear.x = distance/2;
	    move.angular.z = 0;
	    motor_pub.publish(move);
	    ROS_INFO("%f",distance);
	}
	else {
	    move.linear.x = 0;
	    move.angular.z = 0;
	    motor_pub.publish(move);
	    if (current_pose != nb_poses-1)
	    	current_pose++;
	}
   }
   else {
	move.linear.x = 0;
	if (std::abs(desired_angle) > 0.20) {
	    if (desired_angle > 0)
	    	move.angular.z = 0.6;
	    else
		move.angular.z = -0.6;
	}
	else {
	    if (desired_angle > 0)
	    	move.angular.z = 0.5;
	    else
		move.angular.z = -0.5;
	}
	ROS_INFO("%f ; %f", desired_angle, distance);
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
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.orientation.w = 1;
    poses.poses.push_back(pose);
    nb_poses = 1;
    current_pose = 0;
    while(ros::ok()) {
	ros::spin();
    }
}
