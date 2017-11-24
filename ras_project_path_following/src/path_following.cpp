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
		int closest_line;
		int nb_poses;
		int angle_ok;
		int distance_ok;
		ros::Time teleopTime;
		float (*vectors)[2];
		float D;
		float speed;
		bool first;
        bool updated;
   	//Desired velocity is relative to the traveling distance min maxed
   	double v_max, v_min;
   	//Desired angular velocity is relative to the angle differnce min maxed
   	double w_max,w_min;

   	//gain in the prportionality term
   	double vGain,wGain;

   	//Tolerance for angular error and distance error
   	double theta_error, distance_error;

	public:
		PathFollowing() : angle_ok(0), distance_ok(0), updated(false)
	{
		theta_error = 0.1; //0.05
		distance_error = 0.01; //0.01
		v_min = 0.1; //0.01
		v_max = 0.5; // 0.2
		vGain = 0.5; //  1
		D = 0.3;
		speed = 0.1;
		w_min = 0.1; // previosu was 0.5, testing 0.4
		w_max = 0.6; // previous was 06
		wGain = 0.6; // from the fact that pi = 3.14 , pi*0.2 = 0.628 roughly
		first = false;
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
  	    closest_line = 0;
  	    nb_poses = poses.poses.size();
     	    updated = true;
	    first = true;
	    vectors = new float[nb_poses-1][2];
	    for (int i = 0; i < nb_poses-1; i++)
	    {
		if (i == 0)
		    ROS_INFO("x1: %f - y1: %f ; x2: %f - y2: %f",poses.poses[0].position.x,poses.poses[0].position.y,poses.poses[1].position.x,poses.poses[1].position.y);
	     	float length = std::sqrt(std::pow(poses.poses[i+1].position.x - poses.poses[i].position.x,2) + std::pow(poses.poses[i+1].position.y - poses.poses[i].position.y,2));
	    	float x = poses.poses[i+1].position.x - poses.poses[i].position.x;
	     	float y = poses.poses[i+1].position.y - poses.poses[i].position.y;
		if (i == 0)
		    ROS_INFO("Length: %f ; x: %f ; y: %f",length,x,y);
	     	vectors[i][0] = x/length;
	     	vectors[i][1] = y/length;
		if (i == 0)
		    ROS_INFO("Vecteur - x: %f ; y: %f",vectors[0][0], vectors[0][1]);
	    }
	}
}

void PathFollowing::odometryCallback (const geometry_msgs::PoseStamped::ConstPtr& msg) {
   if(!updated){return;} // should one add at least 0 velocity publish here?
   geometry_msgs::PoseStamped robot_pose = *msg;
   geometry_msgs::Twist move;
   double y_current = robot_pose.pose.position.y;
   double x_current = robot_pose.pose.position.x;
   //double final_pose= poses.poses[current_pose].orientation.z; // Z NOT QUATERINO, ITS YAW
   //double theta = atan2(y_desired-y_current, x_desired-x_current);
   float PI = std::acos(-1);
   double roll,pitch,yaw;
   tf::Quaternion q(robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z ,robot_pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll,pitch,yaw);
   double theta_current =yaw;
   // Determine closest point to the path
   float smallest_distance = -1;
   float distance_line;
   for (int i = 0; i < nb_poses - 1; i++)
   {
	float x1 = poses.poses[i].position.x;
	float y1 = poses.poses[i].position.y;
	float x2 = poses.poses[i+1].position.x;
	float y2 = poses.poses[i+1].position.y;
        float projection_length = std::max(0.0, std::min(1.0, (x_current - poses.poses[i].position.x)*vectors[i][0] + (y_current - poses.poses[i].position.y)*vectors[i][1]));
	float projection[2] = {poses.poses[i].position.x + projection_length*vectors[i][0], poses.poses[i].position.y + projection_length*vectors[i][1]};
	float distance = std::sqrt(std::pow(x_current - projection[0],2)+std::pow(y_current - projection[1], 2));
	if (distance < smallest_distance || smallest_distance == -1)
	{
	    smallest_distance = distance;
	    closest_line = i;
	    distance_line = projection_length;
	}
   }
//   float hypotenuse = std::sqrt(std::pow(poses.poses[closest_line].position.y-y_current, 2) + std::pow(poses.poses[closest_line].position.x-x_current, 2));
//   float distance_line = std::sqrt(std::pow(hypotenuse,2) - std::pow(smallest_distance, 2));
//   float distance_line = -(2.0*vectors[closest_line][0]*(poses.poses[closest_line].position.x-x_current)+2.0*vectors[closest_line][1]*(poses.poses[closest_line].position.y-y_current))/(2.0*(std::pow(vectors[closest_line][0],2)+std::pow(vectors[closest_line][1],2)));
   float closest_point[2];
   closest_point[0] = poses.poses[closest_line].position.x + distance_line*vectors[closest_line][0];
   closest_point[1] = poses.poses[closest_line].position.y + distance_line*vectors[closest_line][1];
   if (first)
   {
   	ROS_INFO("Closest point on %d, x: %f; y: %f",closest_line,closest_point[0], closest_point[1]);
   }
   // Find goal point which is a distance D away from closest_point on path
   float goal_point[2];
   int i;
   for (i = closest_line; i < nb_poses-1; i++)
   {
	if (first)
	    ROS_INFO("Vector %d, x: %f; y: %f",i ,vectors[i][0], vectors[i][1]);
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
	    float segment = std::sqrt(std::pow(poses.poses[i+1].position.y - poses.poses[i].position.y, 2)+std::pow(poses.poses[i+1].position.x - poses.poses[i].position.x,2));
	    if (i == closest_line)
	    {
		if (z1 > distance_line && z1 <= segment && (z1 < z2 || z2 <= distance_line))
		{
		    goal_point[0] = poses.poses[i].position.x + z1*vectors[i][0];
		    goal_point[1] = poses.poses[i].position.y + z1*vectors[i][1];
		    if (first)
		    	ROS_INFO("Goal Point - x: %f, y: %f",goal_point[0],goal_point[1]);
		    break;
		}
		else if (z2 > distance_line && z1 <= segment && (z2 < z1 || z1 <= distance_line))
		{
		    goal_point[0] = poses.poses[i].position.x + z2*vectors[i][0];
		    goal_point[1] = poses.poses[i].position.y + z2*vectors[i][1];
		    if (first)
		    	ROS_INFO("Goal Point - x: %f, y: %f", goal_point[0],goal_point[1]);
		    break;
		}
	    }
	    else
	    {
		if (z1 >= 0 && z1 <= segment && (z1 < z2 || z2 < 0))
		{
		    goal_point[0] = poses.poses[i].position.x + z1*vectors[i][0];
		    goal_point[1] = poses.poses[i].position.y + z1*vectors[i][1];
		    break;
		}
		else if (z2 >= 0 && z2 <= segment && (z2 < z1 || z1 < 0))
		{
		    goal_point[0] = poses.poses[i].position.x + z2*vectors[i][0];
		    goal_point[1] = poses.poses[i].position.y + z2*vectors[i][1];
		    break;
		}
	    }
	}
   }
   if (i == nb_poses)
   {
	goal_point[0] = poses.poses[i].position.x;
	goal_point[1] = poses.poses[i].position.y;
   }
   // Now that we have the goal point, let's change the coordinates of this point into robot's coordinates
   theta_current -= PI/2.0;
   float xgv = (goal_point[0] - x_current)*std::cos(theta_current) + (goal_point[1] - y_current)*std::sin(theta_current);
   float ygv = -(goal_point[0] - x_current)*std::sin(theta_current) + (goal_point[1] - y_current)*std::cos(theta_current);
   if (first)
   {
	ROS_INFO("Robot - x: %f, y: %f",x_current, y_current);
	ROS_INFO("Goal point robot - x: %f, y: %f",xgv,ygv);
   }
   // Now we should calculate the curvature of the circle we want to follow to the point
   float curvature = 2*xgv/std::pow(D,2);
   // Then we calculate the angular_velocity necessary to follow the right trajectory
   float time = (2*PI)/(curvature*speed);
   move.linear.x = speed;
   move.angular.z = -2*PI/time;
   if (first)
   {
	ROS_INFO("Linear: %f, Angular: %f", move.linear.x, move.angular.z);
   }
   first = false;
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

