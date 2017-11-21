#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"

float readings[360];

void scanCallback ( const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for (int i = 0; i < 360; i++)
    {
    	readings[i] = msg->ranges[i];
    }
}

float* robot_distances (float radius)
{
    float PI = std::acos(-1);
    float pos_x_lidar = 0.024;
    float pos_y_lidar = 0.093;
    float dist = std::sqrt(std::pow(pos_x_lidar,2)+std::pow(pos_y_lidar,2));
    float alpha = std::acos(pos_x_lidar/dist);
    float* distances = new float[360];
    for (int i = 0; i < 360; i++) {
	float theta = i*PI/180.0;
	float beta, epsilon;
	if (0 < theta && theta < alpha)
	{
	    beta = std::asin(dist*sin(PI+theta-alpha)/radius);
	    epsilon = PI - beta - (PI+theta-alpha);
	}
	else if (alpha < theta && theta < PI + alpha)
	{
	    beta = std::asin(dist*sin(PI + alpha - theta)/radius);
	    epsilon = PI - beta - (PI + alpha - theta);
	}
	else
	{
	    beta = std::asin(dist*std::sin(theta - PI - alpha)/radius);
	    epsilon = PI - beta - (theta - PI - alpha);
	}
	distances[359-i] = dist*std::sin(epsilon)/std::sin(beta);
    }
    return distances;
}

float* obstacles_distances (float sides_radius, float front_radius, float front_angles)
{
    float PI = std::acos(-1);
    float pos_x_lidar = 0.024;
    float pos_y_lidar = 0.093;
    float dist = std::sqrt(std::pow(pos_x_lidar,2)+std::pow(pos_y_lidar,2));
    float alpha = std::acos(pos_x_lidar/dist);
    float* distances = new float[360];
    for (int i = 0; i < 360; i++) {
	float theta = i*PI/180.0;
	float beta, epsilon, radius;
	if ((0.0 <= i && i <= front_angles/2.0) || ((360.0 - front_angles/2.0) <= i && i <= 360.0))
	{
	    radius = front_radius;
	}
	else
	    radius = sides_radius;
	if (0 < theta && theta < alpha)
	{
	    beta = std::asin(dist*sin(PI+theta-alpha)/radius);
	    epsilon = PI - beta - (PI+theta-alpha);
	}
	else if (alpha < theta && theta < PI + alpha)
	{
	    beta = std::asin(dist*sin(PI + alpha - theta)/radius);
	    epsilon = PI - beta - (PI + alpha - theta);
	}
	else
	{
	    beta = std::asin(dist*std::sin(theta - PI - alpha)/radius);
	    epsilon = PI - beta - (theta - PI - alpha);
	}
	distances[359-i] = dist*std::sin(epsilon)/std::sin(beta);
    }
    return distances;
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_collision_detector");
    ros::NodeHandle n;
    ros::Publisher collision = n.advertise<std_msgs::Bool>("/obstacles",5);
    ros::Subscriber laser = n.subscribe("scan", 1000, scanCallback);
    std_msgs::Bool obstacles;
    obstacles.data = false;
    float radius = 0.121;
    float front_radius = 0.25;
    float sides_radius = 0.14;
    float* robot = robot_distances (radius);
    float* zone = obstacles_distances ( sides_radius, front_radius, 60.0);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
	if (obstacles.data == true)
	    obstacles.data = false;
	for (int i = 0; i < 360; i++)
	{
	    if (robot[i] < readings[i] && readings[i] < zone[i])
	    {
		ROS_INFO("theta : %d", i);
		ROS_INFO("robot : %f", robot[i]);
		ROS_INFO("zone : %f", zone[i]);
		ROS_INFO("readings : %f", readings[i]);
		obstacles.data = true;
		break;
	    }
	}
	collision.publish(obstacles);
	loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
