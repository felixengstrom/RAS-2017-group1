#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <cmath>

double lin_vel_;
double ang_vel_;
const static int PI = acos(-1);


void teleopCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n_;

    ros::Subscriber teleop_sub;
    ros::Publisher est_vel_pub;
    
    teleop_sub = n_.subscribe("motor_teleop/twist", 10, teleopCallback);
    est_vel_pub = n_.advertise<geometry_msgs::Twist>("est_robot_vel/twist", 10);
    sim_vel_pub = n_.advertise<geometry_msgs::Twist>("simualiton/velocity", 10);
    
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
            
            geometry_msgs::Twist mes;

            // Todo: add noice to simulate error inm essurements
            mes.linear.x = lin_vel_;
            mes.angular.z = ang_vel_;

            est_vel_pub.publish(mes);

            // Todo: add code to publish simulated postion on map

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
