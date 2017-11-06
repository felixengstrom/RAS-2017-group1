#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "phidgets/motor_encoder.h"

const static float PI = acos(-1);
class OdometryNode
{
    private:
        ros::NodeHandle n_;
        ros::Publisher odom_pub; 
        ros::Subscriber sub;
        tf::Transform transform;
        tf::TransformBroadcaster br;
        tf::Quaternion q;
        ros::Time time;
        double current_x, current_y, current_omega;

    public:
        OdometryNode() 
        {
            time = ros::Time::now();
            current_x = 0.22;
            current_y = 0.22;
            current_omega = PI/2;
            n_ = ros::NodeHandle();
            sub = n_.subscribe("est_robot_vel/twist", 10, &OdometryNode::VelocityCallback, this);
	    odom_pub = n_.advertise<geometry_msgs::Pose2D>("robot/pose", 1000);
        }
        void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg );
};

void OdometryNode::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - time).toSec();
    time = current_time;
    double v = msg->linear.x; 
    double omega = msg->angular.z; 
    current_x = current_x + cos(current_omega)*v*dt;
    current_y = current_y + sin(current_omega)*v*dt;
    current_omega = current_omega + omega*dt;
    if (current_omega > PI) {
	current_omega = fmod(current_omega,PI)-PI;
    }
    geometry_msgs::Pose2D pose;
    pose.x = current_x;
    pose.y = current_y;
    pose.theta = current_omega;
    odom_pub.publish(pose);
    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    q.setRPY(0, 0, current_omega);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot"));
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "odometry");
    OdometryNode on  = OdometryNode();
    ros::spin();
    return 0;
}
