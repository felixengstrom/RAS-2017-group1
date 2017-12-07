#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <random>


const static float PI = acos(-1);

class MotorMockup
{
    private:
        float current_x; 
        float current_y;
        float current_omega;
        double lin_vel;
        double ang_vel;
        bool hold;
        ros::Time last;
        ros::NodeHandle n_;
        ros::Subscriber teleop_sub;
        ros::Subscriber hold_sub;
        ros::Subscriber initialpose_sub;
        ros::Publisher est_vel_pub;
        geometry_msgs::PoseStamped truepose;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        std::default_random_engine rng;
        std::normal_distribution<double> noise;

    public: 
        MotorMockup(double mean, double var);
        void updatePosAndSendTransform();
        void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void holdCallback(const std_msgs::Bool::ConstPtr& msg);
        void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

};

MotorMockup::MotorMockup(double mean, double var)
    : n_(), truepose(), br(), transform(), q(),noise(mean, var), hold(false), rng()
{
    teleop_sub = n_.subscribe("motor_teleop/twist", 1, &MotorMockup::teleopCallback, this);
    hold_sub = n_.subscribe("hold", 1, &MotorMockup::holdCallback, this);
    initialpose_sub = n_.subscribe("initialpose", 1, &MotorMockup::initialposeCallback, this);
    est_vel_pub = n_.advertise<geometry_msgs::TwistStamped>("est_robot_vel/twist", 1);
    last = ros::Time::now();
    current_x = 0.22; 
    current_y = 0.22;
    current_omega = PI/2.0;

    q.setRPY(0, 0, current_omega);
    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

}

void MotorMockup::teleopCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel = msg->linear.x;
    ang_vel = msg->angular.z;
}

void MotorMockup::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    last = msg->header.stamp;
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    tf::Quaternion q_;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q_);
    current_omega = tf::getYaw(q_);
}

void MotorMockup::holdCallback(const std_msgs::Bool::ConstPtr& msg)
{
    hold = msg->data;
}

void MotorMockup::updatePosAndSendTransform()
{
    // Broadcast on tf
    ros::Time current  = ros::Time::now();
    ros::Duration elapsed = current-last;
    last = current;
    if (!hold)
    {
        current_x = current_x +elapsed.toSec()*lin_vel*cos(current_omega + ang_vel*0.05);
        current_y = current_y +elapsed.toSec()*lin_vel*sin(current_omega + ang_vel*0.05);
        current_omega = current_omega + ang_vel*elapsed.toSec();
    }
        
    q.setRPY(0, 0, current_omega);
    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "truepos"));
    geometry_msgs::TwistStamped mes;

    mes.header.stamp = ros::Time::now();
    mes.twist.linear.x = lin_vel + lin_vel*noise(rng);
    mes.twist.angular.z = ang_vel+ ang_vel*noise(rng);
    est_vel_pub.publish(mes);

}

int main (int argc, char **argv)
{

    ros::init(argc, argv, "motor_mockup");
    ros::NodeHandle n("~");

    double mean;
    n.param<double>("mean", mean, 0.0);
    double var;
    n.param<double>("var", var, 0.3);

    MotorMockup mm(mean, var);
    ros::Rate rate(100);
    while(ros::ok())
    {   
        ros::spinOnce();
        mm.updatePosAndSendTransform();
        rate.sleep();
    }
    return 0;
}
