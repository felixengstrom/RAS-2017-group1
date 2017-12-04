#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Bool.h>
class ObjectAddition
{
	private:
		ros::NodeHandle n;
		//ros::Publisher motor_pub;
		ros::Subscriber position_sub;
		ros::Subscriber classification_sub;
		geometry_msgs::PointStamped object_point;
		int object_classification;
		std::list<std::array<float,3>> classified_objects;
		char* filename;

	public:
	ObjectAddition()
	{
		n = ros::NodeHandle();
    		position_sub = n.subscribe("/map/objectCoord", 10, &ObjectAddition::positionCallback,this);
    		classification_sub = n.subscribe("/camera/object_class", 10, &ObjectAddition::classificationCallback,this);
		object_point = NULL;
		object_classification = 0;
		classified_objects = NULL;
		filename = "classified_objects.txt";
	}
		void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
		void classificationCallback(const std_msgs::String::ConstPtr& msg);
		void object_add(void);
		int classification_string_to_int(char* classification);
		float classification_to_radius(int classification);
};

void ObjectAddition::positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
	object_point = *msg;
	if (object_classification != 0)
	    this.object_add();
	return;
}

void ObjectAddition::classificationCallback (const std_msgs::String::ConstPtr& msg) {
	object_classification = classification_string_to_int(msg->data);
	if (object_point != NULL)
	    this.object_add();
	return;
}

int classification_string_to_int(char* classification)
{
	if (classification == "An object")
		return 1;
	else if (classification == "Red Cube")
		return 2;
	else if (classification == "Red Hollow Cube")
		return 3;
	else if (classification == "Blue Cube")
		return 4;
	else if (classification == "Green Cube")
		return 5;
	else if (classification == "Yellow Cube")
		return 6;
	else if (classification == "Yellow Ball")
		return 7;
	else if (classification == "Red Ball")
		return 8;
	else if (classification == "Red Cylinder")
		return 9;
	else if (classification == "Green Cylinder")
		return 10;
	else if (classification == "Green Hollow Cube")
		return 11;
	else if (classification == "Blue Triangle")
		return 12;
	else if (classification == "Purple Cross")
		return 13;
	else if (classification == "Purple Star")
		return 14;
	else if (classification == "Orange Cross")
		return 15;
	else if (classification == "Patric")
		return 16;
	else if (classification == "Battery")
		return 17;
	else if (classification == "Booby Trap")
		return 18;
	else
		return 0;
}

float classification_to_radius(int classification)
{
	switch(classification)
	{
		case 2:
		case 4:
		case 5:
		case 6:
			return 0.055/2.0;
		case 3:
		case 11:
			return 0.052/2.0;
		case 7:
		case 8:
			return 0.045/2.0;
		case 9:
		case 10:
			return 0.046/2.0;
		case 12:
			return 0.048/2.0;
		case 13:
		case 14:
		case 15:
		case 16:
			return 0.05/2.0;
		default:
			return 0.0;
	}
}

void object_add(void)
{
	float object_radius = classification_to_radius(object_classification);
	bool new_object = true;
	for (const std::array<float, 3>& object : classified_objects)
	{
		float distance = std::sqrt(std::pow(object[0] - object_position.point.x,2)+std::pow(object[1] - object_position.point.y,2));
		if (object_radius + classification_to_radius(object[2]) > distance)
		{
			new_object = false;
			break;
		}
	}
	if (new_object)
	{
		std::array<float, 3> object;
		object[0] = object_position.point.x;
		object[1] = object_position.point.y;
		object[2] = (float) object_classification;
		classified_objects.push_back(object);
		add_to_file(object);
		add_to_map(object);
	}
}

void add_to_file(std::array<float, 3> object)
{

}

void add_to_map(std::array<float, 3> object)
{
	float deg_to_rad = std::acos(-1)/180.0;
	for (float i = 0.0; i < 360.0; i += 30.0)
	{
		
	}
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
   ROS_INFO("YGV : %f",ygv);
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

