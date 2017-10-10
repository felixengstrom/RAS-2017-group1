#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
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
            current_x = current_y = current_omega = 0;
            n_ = ros::NodeHandle();
            sub = n_.subscribe("est_robot_vel/twist", 10, &OdometryNode::VelocityCallback, this);
        }
        void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg );
};


void OdometryNode::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - time).toSec();
    time = current_time;
    double v = 6*msg->linear.x; 
    double omega =6* msg->angular.z; 
    current_x = current_x + cos(current_omega)*v*dt;
    current_y = current_y + sin(current_omega)*v*dt;
    current_omega = current_omega + omega*dt;
    ROS_INFO("x:%f y:%f, omega:%f", current_x, current_y, current_omega);
    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    q.setRPY(0, 0, current_omega);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "odometry");
    OdometryNode on  = OdometryNode();
    ros::spin();
    return 0;
}
