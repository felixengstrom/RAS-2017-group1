#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
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
   	//Desired velocity is relative to the traveling distance min maxed
   	double v_max, v_min;
   	//Desired angular velocity is relative to the angle differnce min maxed
   	double w_max,w_min;

   	//gain in the prportionality term
   	double vGain,wGain;

   	//Tolerance for angular error and distance error
   	double theta_error, distance_error;

	double v_prev; //previous velocity
	public:
		PathFollowing() : angle_ok(0), distance_ok(0), updated(false)
	{
		theta_error = 0.05;
		distance_error = 0.01;
		v_min = 0.01; //from previous value
		v_max = 0.2; // non tested 0.3 m/s
		vGain = 1; // 

		w_min = 0.1; // previosu was 0.5, testing 0.4
		w_max = 0.6; // previous was 06
		wGain = 0.6; // from the fact that pi = 3.14 , pi*0.2 = 0.628 roughly
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
   if(!updated){return;} // should one add at least 0 velocity publish here?
   geometry_msgs::PoseStamped robot_pose = *msg;
   geometry_msgs::Twist move;

   double y_desired = poses.poses[current_pose].position.y;
   double y_current = robot_pose.pose.position.y;
   double x_desired = poses.poses[current_pose].position.x;
   double x_current = robot_pose.pose.position.x;
   double theta = atan2(y_desired-y_current, x_desired-x_current);
   double roll,pitch,yaw;
   tf::Quaternion q(robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z ,robot_pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll,pitch,yaw);
   double theta_current =yaw;
   float desired_angle = fmod((theta-theta_current), M_PI);//fmod = modulus for double
   ROS_INFO_STREAM("current X: " << x_current << " current Y: " << y_current << "des X: " << x_desired << " des Y: " << y_desired);
   ROS_INFO_STREAM("Angle desired: "<< desired_angle << " theta = " << theta << " current = " << theta_current);
   double distance = std::sqrt(std::pow(x_current-x_desired,2)+std::pow(poses.poses[current_pose].position.y-robot_pose.pose.position.y,2));


// 
   //if (std::abs(desired_angle) < 0.05 || distance <= 0.01) {

   //If ONE of the error (or both) is BELOW the treshhold error
  //if (std::abs(desired_angle) < theta_error || distance <= distance_error) {
  //With the above logic the pose is not determined upon end position!
  if (std::abs(desired_angle) < theta_error  || distance <= distance_error) {


	  //If the distance is ABOVE threshhold
  	if (distance > distance_error) {

	    move.linear.x = std::max(std::min(v_max,vGain*distance),v_min);
	    move.angular.z = 0; // Straight movement
	    v_prev = move.linear.x;
	    motor_pub.publish(move);
	    ROS_INFO("%f",distance);
	}
	  //if the distance is Below treshhold, with previous if this is when it is close enough to goal
	else {
	    move.linear.x = 0;
	    move.angular.z = 0;
	    v_prev = move.linear.x;
	    motor_pub.publish(move);
	    if (current_pose != nb_poses-1)
	    	current_pose++;
            else updated = false;
	}
   }
   else {
	move.linear.x = 0; // the zero can give a hickup movement..
	//if (distance > distance_error) {move.linear.x =v_min/2;	  v_prev = move.linear.x; }  
	//else {move.linear.x = 0;v_prev = move.linear.x;}
	    if (desired_angle > 0)
	    	move.angular.z = std::max(std::min(w_max,wGain*atan(desired_angle)),w_min);
	    else
		move.angular.z = std::min(std::max(wGain*atan(desired_angle),-1*w_max), -1*w_min);
	}
	ROS_INFO("%f ; %f", desired_angle, distance);
	motor_pub.publish(move);
   
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_following");
    PathFollowing P;
    ros::Rate r(50);
    while(ros::ok()) {
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}
