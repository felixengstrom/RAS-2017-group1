#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "phidgets/motor_encoder.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <cmath>

double lin_vel_;
double ang_vel_;
const static int PI = acos(-1);

class EncoderListener
{
    private:
        ros::Subscriber left_encoder_sub;
        ros::Subscriber right_encoder_sub;


    public:
        int last_change_right;
        int last_change_left;
        ros::Time last_reading_left;
        ros::Time last_reading_right;
        double duration_left;
        double duration_right;
        EncoderListener( ros::NodeHandle& n_);
        void LeftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
        void RightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg);
};

EncoderListener::EncoderListener( ros::NodeHandle& n_ )
{
    n_ = ros::NodeHandle();
    left_encoder_sub = n_.subscribe("/left_motor/encoder", 1,
                                    &EncoderListener::LeftEncoderCallback,
                                            this);

    right_encoder_sub = n_.subscribe("/right_motor/encoder", 1,
                                    &EncoderListener::RightEncoderCallback,
                                            this);
    last_change_right = 0;
    last_change_left = 0;
    last_reading_left;
    last_reading_right;
}

void EncoderListener::LeftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    last_change_left = msg->count_change;
    duration_left = (last_reading_left - msg->header.stamp).toSec();
    last_reading_left = msg->header.stamp;
    ROS_INFO("encoder sum %i", msg->count );
}

void EncoderListener::RightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    last_change_right = msg->count_change;
    duration_right = (last_reading_right - msg->header.stamp).toSec();
    last_reading_right = msg->header.stamp;
}

void teleopCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n_;
    //ros::Subscriber twist_sub;
    ros::Subscriber teleop_sub;
    ros::Publisher vel_left_pub;
    ros::Publisher vel_right_pub;
    ros::Publisher est_vel_pub;
    
    EncoderListener listener = EncoderListener(n_);

    teleop_sub = n_.subscribe("motor_teleop/twist", 10, teleopCallback);
    vel_left_pub = n_.advertise<std_msgs::Float32>("left_motor/cmd_vel", 10);
    vel_right_pub = n_.advertise<std_msgs::Float32>("right_motor/cmd_vel", 10);
    est_vel_pub = n_.advertise<geometry_msgs::Twist>("est_robot_vel/twist", 10);
    
    ros::Rate loop_rate(10);

    double dt = 0.1;
    const double control_frequency =100; //10; //still of by about 4
    const double ticks_per_rev = 3591.84;//900; //3591.84;
    const double base_ = 0.238;
    const double wheel_radius_ = 0.036;

    //Controller parameters
    double int_err_left = 0.0;
    double alpha_left = 7.5;//7.5/5;//12
    double beta_left = 30.5;//30.0/5;//20

    double int_err_right = 0.0;
    double alpha_right = 7.5;///5;//12
    double beta_right = 30.0;///5;//20

    while(ros::ok())
    {

        // Desired wheels angular velocities
		
            float desired_wl = (lin_vel_ - base_*ang_vel_/2)/wheel_radius_;
            float desired_wr = (lin_vel_ + base_*ang_vel_/2)/wheel_radius_;
            ROS_INFO("desired left : %f", desired_wl);
            ROS_INFO("desired right : %f", desired_wr);
            // Code for calculating estimated current velocity based on encoders

	        int n=5, i, sum_left=0, sum_right=0;
            int last_change_left = (listener.last_change_left);
            int last_change_right = -(listener.last_change_right);

            ROS_INFO("encoder duration left : %f", listener.duration_left);
            ROS_INFO("escoder duration right : %f", listener.duration_right);            

            ROS_INFO("encoder change left : %i", last_change_left);
            ROS_INFO("escoder change right : %i", last_change_right);

            float est_wl = last_change_left*2*PI*control_frequency/ticks_per_rev;
            float est_wr = last_change_right*2*PI*control_frequency/ticks_per_rev;

            ROS_INFO("est left : %f", est_wl);
            ROS_INFO("est right : %f", est_wr);

            float v = wheel_radius_*(est_wl + est_wr)/2;
            float omega = wheel_radius_*(est_wr-est_wl)/base_;

            //ROS_INFO("current omega:%f, current v:%f", omega, v);

            //Controller
        
            float error_left = desired_wl - est_wl;
            float error_right = desired_wr - est_wr;

            ROS_INFO("error left : %f", error_left);
            ROS_INFO("error right : %f", error_right);

            int_err_left += error_left*dt;
            int_err_right += error_right*dt;

	//set max PWM values 100
            float pwm_left_float = std:: max(-100.0, (std::min(100.0,(1*desired_wl + alpha_left * error_left + beta_left * int_err_left))));
            float pwm_right_float =  std::max(-100.0, (std::min(100.0, (1*desired_wr + alpha_right * error_right + beta_right * int_err_right))));
            if (abs(pwm_left_float)>=100.0)
	{
		int_err_left -=error_left*dt;
	}
	   if  (abs(pwm_right_float)>=100.0)
	{
		int_err_right -=error_right*dt;
	}
            ROS_INFO("integral error left: %f", int_err_left);
	    ROS_INFO("integral error right: %f", int_err_right);

            ROS_INFO("estimated v:%f, estimated omega:%f", v, omega);
            ROS_INFO("wanted v:%f, wanted omega:%f", lin_vel_, ang_vel_);

            //Publishing PWM to left and right motors
            std_msgs::Float32 msg1;
            std_msgs::Float32 msg2;
            
	    if (lin_vel_ == 0 && ang_vel_ ==0)
	{
	    pwm_left_float = 0;
		msg1.data = pwm_left_float;
	    pwm_right_float = 0;
		msg2.data = pwm_right_float;
	}
            else 
	{
		msg1.data = pwm_left_float;
           	msg2.data = pwm_right_float;
	}
ROS_INFO ("PWM left: %f", pwm_left_float);
ROS_INFO ("PWM right:%f", pwm_right_float);
            vel_left_pub.publish(msg1);
            vel_right_pub.publish(msg2);

            //Publishing estimated velosity of robot to calc odometry  
            
            geometry_msgs::Twist mes;

            mes.linear.x = v;
            mes.angular.z = omega;

            est_vel_pub.publish(mes);

            lin_vel_ = ang_vel_ = 0;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
