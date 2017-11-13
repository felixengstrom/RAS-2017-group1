#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
class PathFollowing
{
	private:
		ros::NodeHandle n;
		ros::Publisher motor_pub;
		ros::Subscriber teleop_sub;
		ros::Subscriber odometry_sub;
		geometry_msgs::PoseArray poses;
		int current_pose;
		int nb_poses;
		int angle_ok;
		int distance_ok;
		ros::Time teleopTime;
        bool updated;

	public:
		PathFollowing() : angle_ok(0), distance_ok(0), updated(false)
	{
		n = ros::NodeHandle();
		teleopTime = ros::Time::now();
    		teleop_sub = n.subscribe("/pose_teleop", 10, &PathFollowing::teleopCallback,this);
    		odometry_sub = n.subscribe("/robot/pose", 10, &PathFollowing::odometryCallback,this);
    		motor_pub = n.advertise<geometry_msgs::Twist>("motor_teleop/twist", 10);
	}
		void teleopCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
		void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

void PathFollowing::teleopCallback (const geometry_msgs::PoseArray::ConstPtr& msg) {
	ros::Time cur = msg->header.stamp;
	if(cur != teleopTime)
	{	
	teleopTime = cur;
  	 poses = *msg;
  	 current_pose = 0;
  	 nb_poses = poses.poses.size();
     updated = true;
	}
}

void PathFollowing::odometryCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
   if(!updated){return;}
   geometry_msgs::PoseStamped robot_pose = *msg;
   geometry_msgs::Twist move;
   double y_desired = poses.poses[current_pose].position.y;
   double y_current = robot_pose.pose.position.y;
   double x_desired = poses.poses[current_pose].position.x;
   double x_current = robot_pose.pose.position.x;
   double theta = atan2(y_desired-y_current, x_desired-x_current);
   float desired_angle = theta-(atan2(robot_pose.pose.orientation.z,robot_pose.pose.orientation.w)*2.0);
   ROS_INFO("Angle desired: %f", desired_angle);
   double distance = std::sqrt(std::pow(x_current-x_desired,2)+std::pow(poses.poses[current_pose].position.y-robot_pose.pose.position.y,2));
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
        else updated = false;
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
    PathFollowing P;
    ros::Rate r(10);
    while(ros::ok()) {
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}
