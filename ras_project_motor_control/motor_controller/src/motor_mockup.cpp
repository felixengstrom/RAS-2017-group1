#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>

double lin_vel_;
double ang_vel_;
const static float PI = acos(-1);


void teleopCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
}

int main (int argc, char **argv)
{
    std::default_random_engine rng;
    std::normal_distribution<double> noise(0.0, 0.1);

    ros::init(argc, argv, "motor_controller_mockup");
    ros::NodeHandle n_;

    ros::Subscriber teleop_sub;
    ros::Publisher est_vel_pub;
    
    teleop_sub = n_.subscribe("motor_teleop/twist", 10, teleopCallback);
    est_vel_pub = n_.advertise<geometry_msgs::Twist>("est_robot_vel/twist", 10);

    geometry_msgs::PoseStamped truepose;
    tf::TransformBroadcaster br;
    
    tf::Transform transform;
    tf::Quaternion q;
    
    ros::Rate loop_rate(10);

    float current_x = 0.22; 
    float current_y = 0.22;

    float current_omega = PI/2.;

    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    q.setRPY(0, 0, current_omega);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot"));
    ros::Time last  = ros::Time::now();

    ros::Rate rate(10);

    while(ros::ok())
    {   
        ros::Time current  = ros::Time::now();
        ros::Duration elapsed = current-last;
        last = current;
        current_x = current_x +elapsed.toSec()*lin_vel_*cos(current_omega + ang_vel_*0.05);
        current_y = current_y +elapsed.toSec()*lin_vel_*sin(current_omega + ang_vel_*0.05);
        current_omega = current_omega + ang_vel_*0.1;
            
        q.setRPY(0, 0, current_omega);
        transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "truepos"));
        geometry_msgs::Twist mes;

        // Todo: add noice to simulate error inm essurements
        mes.linear.x = lin_vel_ + abs(lin_vel_*noise(rng)*0);
        mes.angular.z = ang_vel_+ ang_vel_*noise(rng)*10;

        est_vel_pub.publish(mes);

        // Todo: add code to publish simulated postion on map

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
