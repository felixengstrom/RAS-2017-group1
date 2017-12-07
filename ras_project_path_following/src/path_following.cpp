#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Bool.h>
class PathFollowing
{
	private:
		ros::NodeHandle n;
		ros::Publisher motor_pub;
		ros::Subscriber teleop_sub;
		ros::Subscriber odometry_sub;
		ros::Subscriber stop_sub;
		ros::Subscriber slowdown_sub;
		ros::Subscriber odomdiff_sub;
		geometry_msgs::PoseArray poses;
		int closest_line;
		int nb_poses;
		ros::Time teleopTime;
		float (*vectors)[2];
		float D;
		float normal_speed;
		float speed;
	bool stop;
	bool first;

	public:
	PathFollowing()
	{
//		theta_error = 0.1; //0.05
//		distance_error = 0.01; //0.01
//		v_min = 0.1; //0.01
//		v_max = 0.5; // 0.2
//		vGain = 0.5; //  1
		D = 0.1;
		normal_speed = 0.09;
		speed = 0.09;
		stop = true;
		first = false;
		n = ros::NodeHandle();
		teleopTime = ros::Time::now();
        teleop_sub= n.subscribe("/pose_teleop", 1, &PathFollowing::teleopCallback,this);
        odometry_sub = n.subscribe("/robot/pose", 1, &PathFollowing::odometryCallback,this);
		stop_sub = n.subscribe("/robot/stop", 1, &PathFollowing::stopCallback,this);
		slowdown_sub = n.subscribe("/pathFollow/slowDown", 1, &PathFollowing::slowdownCallback,this);
		odomdiff_sub = n.subscribe("/robot/odomdiff", 1, &PathFollowing::odomdiffCallback,this);
        motor_pub = n.advertise<geometry_msgs::Twist>("motor_teleop/twist", 1);
	}
		void teleopCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
		void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void stopCallback(const std_msgs::Bool::ConstPtr& msg);
		void slowdownCallback(const std_msgs::Bool::ConstPtr& msg);
		void odomdiffCallback(const std_msgs::Bool::ConstPtr& msg);
};

void PathFollowing::slowdownCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data == true)
	    speed = 0.1;
	else
	    speed = normal_speed;
}

void PathFollowing::odomdiffCallback(const std_msgs::Bool::ConstPtr& msg) {
<<<<<<< HEAD
    ros::Rate r(20);
    for (int i = 0; i<3; i++)
=======
    ros::Rate r(10);
    for (int i = 0; i<12; i++)
>>>>>>> felix_branch
    {
	    geometry_msgs::Twist move;
	    move.linear.x = -0.1;
	    move.angular.z = 0.0;
	    motor_pub.publish(move);
        r.sleep();
    }
    for (int i = 0; i<10; i++)
    {
	    geometry_msgs::Twist move;
	    move.linear.x = 0.0;
	    move.angular.z = 0.0;
	    motor_pub.publish(move);
        r.sleep();
    }
}

void PathFollowing::teleopCallback (const geometry_msgs::PoseArray::ConstPtr& msg) {
	ros::Time cur = msg->header.stamp;
	if(cur != teleopTime)
	{
	    first = true;
	    if (stop == false)
		delete vectors;
	    else
	    	stop = false;
	    teleopTime = cur;
  	    poses = *msg;
  	    closest_line = 0;
  	    nb_poses = poses.poses.size();
	    vectors = new float[nb_poses-1][2];
	    for (int i = 0; i < nb_poses-1; i++)
	    {
	     	float length = std::sqrt(std::pow(poses.poses[i+1].position.x - poses.poses[i].position.x,2) + std::pow(poses.poses[i+1].position.y - poses.poses[i].position.y,2));
	    	float x = poses.poses[i+1].position.x - poses.poses[i].position.x;
	     	float y = poses.poses[i+1].position.y - poses.poses[i].position.y;
	     	vectors[i][0] = x/length;
	     	vectors[i][1] = y/length;
	    }
	}
}

void PathFollowing::stopCallback (const std_msgs::Bool::ConstPtr& msg) {
	stop = msg->data;
}

void PathFollowing::odometryCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
   if(stop)
   {
	geometry_msgs::Twist move;
	move.linear.x = 0;
	move.angular.z = 0;
	motor_pub.publish(move);
	return;
   }
   // should one add at least 0 velocity publish here?
   geometry_msgs::PoseStamped robot_pose = *msg;
   geometry_msgs::Twist move;
   float y_current = robot_pose.pose.position.y;
   float x_current = robot_pose.pose.position.x;
   //double final_pose= poses.poses[current_pose].orientation.z; // Z NOT QUATERINO, ITS YAW
   //double theta = atan2(y_desired-y_current, x_desired-x_current);
   float PI = std::acos(-1);
   double roll,pitch,yaw;
   tf::Quaternion q(robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z ,robot_pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll,pitch,yaw);
   double theta_current =yaw;
   if (first)
   {
	float angle_first_line = std::atan2(vectors[0][1], vectors[0][0]);
	float angle_difference = theta_current - angle_first_line;
	if (angle_difference > 0.2)
	{
	    geometry_msgs::Twist move;
	    move.linear.x = 0;
	    move.angular.z = -0.6;
	    motor_pub.publish(move);
	    return;
	}
	else if (angle_difference < -0.2)
	{
	    geometry_msgs::Twist move;
	    move.linear.x = 0;
	    move.angular.z = 0.6;
	    motor_pub.publish(move);
	    return;
	}
	else
	    first = false;
   }
   // Determine closest point to the path
   float smallest_distance = -1;
   float distance_line;
   for (int i = 0; i < nb_poses - 1; i++)
   {
	float segment_length = std::sqrt(std::pow(poses.poses[i+1].position.x-poses.poses[i].position.x,2)+std::pow(poses.poses[i+1].position.y-poses.poses[i].position.y,2));
	float projection_length = (x_current - poses.poses[i].position.x)*vectors[i][0] + (y_current - poses.poses[i].position.y)*vectors[i][1];
        projection_length = std::max(0.0f, std::min(segment_length, projection_length));
	float projection[2] = {poses.poses[i].position.x + projection_length*vectors[i][0], poses.poses[i].position.y + projection_length*vectors[i][1]};
	float distance = std::sqrt(std::pow(x_current - projection[0],2)+std::pow(y_current - projection[1], 2));
	if (distance < smallest_distance || smallest_distance == -1)
	{
	    smallest_distance = distance;
	    closest_line = i;
	    distance_line = projection_length;
	}
   }
   float closest_point[2];
   closest_point[0] = poses.poses[closest_line].position.x + distance_line*vectors[closest_line][0];
   closest_point[1] = poses.poses[closest_line].position.y + distance_line*vectors[closest_line][1];
   // Find goal point which is a distance D away from closest_point on path
   float goal_point[2];
   int i;
   for (i = closest_line; i < nb_poses-1; i++)
   {
        float a = std::pow(vectors[i][0],2) + std::pow(vectors[i][1],2);
        float b = 2.0*vectors[i][0]*(poses.poses[i].position.x - closest_point[0]) + 2.0*vectors[i][1]*(poses.poses[i].position.y - closest_point[1]);
        float c = std::pow(poses.poses[i].position.y-closest_point[1],2) + std::pow(poses.poses[i].position.x-closest_point[0],2) - std::pow(D,2);
        float delta = std::pow(b,2) - 4.0*a*c;
	if (delta == 0.0 && -b/(2.0*a) >= 0)
	{
	    goal_point[0] = poses.poses[i].position.x + (-b/(2.0*a))*vectors[i][0];
	    goal_point[1] = poses.poses[i].position.y + (-b/(2.0*a))*vectors[i][1];
	    break;
	}
	else if (delta > 0.0)
	{
	    float z1 = (-b-std::sqrt(delta))/(2.0*a);
	    float z2 = (-b+std::sqrt(delta))/(2.0*a);
        float z = std::max(z1, z2);
	    float segment = std::sqrt(std::pow(poses.poses[i+1].position.y - poses.poses[i].position.y, 2)+std::pow(poses.poses[i+1].position.x - poses.poses[i].position.x,2));
		if (z <= segment )
		{
		    goal_point[0] = poses.poses[i].position.x + z*vectors[i][0];
		    goal_point[1] = poses.poses[i].position.y + z*vectors[i][1];
		    break;
		}
	}
   }
   if (i == nb_poses-1)
   {
	goal_point[0] = poses.poses[i].position.x;
	goal_point[1] = poses.poses[i].position.y;
   }
   // Now that we have the goal point, let's change the coordinates of this point into robot's coordinates
   theta_current -= PI/2.0;
   float xgv = (goal_point[0] - x_current)*std::cos(theta_current) + (goal_point[1] - y_current)*std::sin(theta_current);
   float ygv = -(goal_point[0] - x_current)*std::sin(theta_current) + (goal_point[1] - y_current)*std::cos(theta_current);
   // Now we should calculate the curvature of the circle we want to follow to the point
   float curvature = 2*xgv/std::pow(D,2);
   //ROS_INFO("YGV : %f",ygv);
   if (ygv <= 0.01 && goal_point[0] == poses.poses[nb_poses-1].position.x && goal_point[1] == poses.poses[nb_poses-1].position.y)
   {
	float angular_difference = theta_current + PI/2.0 - poses.poses[nb_poses-1].orientation.w;
	if (angular_difference > 0.1)
	{
	    move.linear.x = 0;
	    move.angular.z = -0.6;
	}
	else if (angular_difference < -0.1)
	{
	    move.linear.x = 0;
	    move.angular.z = 0.6;
	}
	else
	{
	    move.linear.x = 0;
	    move.angular.z = 0;
	    stop = true;
	    delete vectors;
	}
   }
   else if (xgv == 0.0)
   {
       move.linear.x = speed;
       move.angular.z = 0;
   }
   else
   {
   // Then we calculate the angular_velocity necessary to follow the right trajectory
       float time = (2*PI)/(curvature*speed);
       move.linear.x = speed;
       move.angular.z = -2*PI/time;
   }
   motor_pub.publish(move);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_following");
    PathFollowing P;
    ros::Rate r(100);
    while(ros::ok()) {
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}

