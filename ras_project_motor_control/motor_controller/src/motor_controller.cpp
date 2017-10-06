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
}

void EncoderListener::LeftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    last_change_left = msg->count_change;
}

void EncoderListener::RightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    last_change_right = msg->count_change;
}


/*void twistCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
    //ROS_INFO("I heard:", msg->linear);
}

void encoderCallback (const phidgets::motor_encoder::ConstPtr& msg)
{
    delta_encoder = msg->count_change;
    //ROS_INFO("I:", msg->delta_encoder2);
}*/

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
    const double control_frequency = 100;
    const double ticks_per_rev = 3591.84;
    const double base_ = 0.238;
    const double wheel_radius_ = 0.036;

    //Controller parameters
    double int_err_left = 0.0;
    double alpha_left = 10.0;
    double beta_left = 5.0;

    double int_err_right = 0.0;
    double alpha_right = 10.0;
    double beta_right = 5.0;

    while(ros::ok())
    {

        // Desired wheels angular velocities
            float desired_wl = (lin_vel_ - base_*ang_vel_/2)/wheel_radius_;
            float desired_wr = (lin_vel_ + base_*ang_vel_/2)/wheel_radius_;
            ROS_INFO("desired left : %f", desired_wl);
            ROS_INFO("desired right : %f", desired_wr);
            // Code for calculating estimated current velocity based on encoders
            int last_change_left = (listener.last_change_left);
            int last_change_right = -(listener.last_change_right);

            ROS_INFO("encoder change left : %i", last_change_left);
            ROS_INFO("escoder change right : %i", last_change_right);

            float est_wl = last_change_left*2*PI*control_frequency/ticks_per_rev;
            float est_wr = last_change_right*2*PI*control_frequency/ticks_per_rev;
            ROS_INFO("est left : %f", est_wl);
            ROS_INFO("est right : %f", est_wr);

            float v = 6*wheel_radius_*(est_wl + est_wr)/2;
            float omega = 6*wheel_radius_*(est_wr-est_wl)/base_;

            //ROS_INFO("current omega:%f, current v:%f", omega, v); 

            //Controller
        	//double  delta_encoder_double = double (delta_encoder);
        
            float error_left = desired_wl - est_wl;
            float error_right = desired_wr - est_wr;

            ROS_INFO("error left : %f", error_left);
            ROS_INFO("error right : %f", error_right);

            int_err_left += error_left*dt;
            int_err_right += error_right*dt;
            

            float pwm_left_float =  alpha_left * error_left + beta_left * int_err_left;
            float pwm_right_float =  alpha_right * error_right + beta_right * int_err_right;
            //int pwm = (int)(pwm_double);
            //float pwmL = (float) desired_wL * 2;
            //float pwmR = (float) desired_wR * 2;
            ROS_INFO("pwm L: %f", pwm_left_float);
            ROS_INFO("pwm R: %f", pwm_right_float);

            ROS_INFO("estimated v:%f, estimated omega:%f", v, omega);
            ROS_INFO("wanted v:%f, wanted omega:%f", lin_vel_, ang_vel_);

            //Publishing PWM to left and right motors
            std_msgs::Float32 msg1;
            std_msgs::Float32 msg2;
            
	    if (lin_vel_ == 0 && ang_vel_ ==0)
	{
		msg1.data = 0;
		msg2.data = 0;
	}
            else 
	{
		msg1.data = pwm_left_float;
           	msg2.data = pwm_right_float;
	}
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
