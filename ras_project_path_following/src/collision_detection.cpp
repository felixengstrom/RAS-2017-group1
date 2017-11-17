#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_collision_detector");
    ros::NodeHandle n;
    ros::Publisher laser_pub_ = n.advertise<sensor_msgs::LaserScan>("/my_scan",5);
    sensor_msgs::LaserScan scan;
    float PI = std::acos(-1);
    scan.header.frame_id = "mylaser";
    scan.angle_min = -PI + (PI/180.0);
    scan.angle_max = PI;
    scan.angle_increment = PI/180.0;
    scan.scan_time = 0.000137764800456;
    scan.range_min = 0.0;
    scan.range_max = 6.0;
    scan.time_increment = scan.scan_time/360.0;
    float dist = std::sqrt(std::pow(0.024,2)+std::pow(0.093,2));
    ROS_INFO("dist: %f",dist);
    float theta1 = std::acos(0.024/dist);
    ROS_INFO("theta1: %f",theta1);
    float r = 0.13;
    scan.intensities = std::vector<float>(360, 0);
    scan.ranges = std::vector<float>(360, 0);
    for (int i = 0; i < 360; i++) {
	scan.intensities[i] = 47.0;
	float theta;
	if (i <= 180)
	    theta = i*PI/180.0;
	else
	    theta = -PI + (i-180)*(PI/180.0);
	ROS_INFO("theta: %f", theta);
	float theta2;
	if (0 < theta < theta1 || 0 > theta > -PI+theta1)
	    theta2 = std::abs(PI-theta1+theta);
	else
            theta2 = std::abs(PI+theta1-theta);
	ROS_INFO("theta2: %f",theta2);
	float theta3 = std::asin(dist*std::sin(theta2)/r);
	ROS_INFO("theta3: %f",theta3);
        float theta4 = PI - theta2 - theta3;
	ROS_INFO("theta4: %f", theta4);
	if (std::abs(theta+theta1) >= PI/180.0)
            scan.ranges[i] = r*std::sin(theta4)/std::sin(theta2);
	else
	    scan.ranges[i] =dist;
	ROS_INFO("range: %f",scan.ranges[i]);
    }
    ros::Rate loop_rate(10);
    while(ros::ok()) {
	laser_pub_.publish(scan);
	loop_rate.sleep();
    }
    ros::spin();

    return 0;
}
