#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "phidgets/motor_encoder.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <cmath>

static double lin_vel_;
static double ang_vel_;
float objDistance;
double alpha_left, beta_left, proportion_left;
double alpha_right, beta_right, proportion_right;

const static int PI = acos(-1);

class EncoderListener
{
    private:
        ros::Subscriber left_encoder_sub;
        ros::Subscriber right_encoder_sub;
        
    public:
        int last_change_right;
        int last_change_left;
		unsigned long last_count_left;
		unsigned long last_count_right;
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
    duration_left = 0;
    duration_right = 0;
    last_reading_left.nsec = 0;
    last_reading_left.sec = 0;
    last_reading_right.nsec = 0;
    last_reading_right.sec = 0;
    last_count_right = 0;
    last_count_left = 0;
}

void EncoderListener::LeftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    if (last_count_left != 0)
    	last_change_left += msg->count - last_count_left;
    else
	last_change_left += msg->count_change;
    //duration_left = (last_reading_left - msg->header.stamp).toSec();
    if ((last_reading_left.sec == 0 && last_reading_left.nsec == 0) || last_reading_left == msg->header.stamp)
    	duration_left = 0.1;
    else
    	duration_left = (msg->header.stamp - last_reading_left).toSec();
    last_reading_left = msg->header.stamp;
    last_count_left = msg->count;
    //ROS_INFO("Encoder left sum %i", msg->count );
}

void EncoderListener::RightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    if (last_count_right != 0)
	last_change_right += msg->count - last_count_right;
    else
    	last_change_right += msg->count_change;
    //duration_right = (last_reading_right - msg->header.stamp).toSec();
    if ((last_reading_right.sec == 0 && last_reading_right.nsec == 0) || last_reading_right == msg->header.stamp)
    	duration_right = 0.1;
    else
    	duration_right = (msg->header.stamp - last_reading_right).toSec();
    last_reading_right = msg->header.stamp;
    last_count_right = msg->count;
    //ROS_INFO("Encoder right sum %i", msg->count );
}

void teleopCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
}

void objPositionMoveCallback(const std_msgs::Float32::ConstPtr& msg)
{
    objDistance = msg->data;
    if (0.0 != objDistance)
    {
        lin_vel_ = 0.06;
        ang_vel_ = 0.0;
        ROS_INFO("Robot is moving closer to the object");
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n_;
    //ros::Subscriber twist_sub;
    ros::Subscriber teleop_sub;
    ros::Subscriber obj_position_move_sub;
    ros::Publisher vel_left_pub;
    ros::Publisher vel_right_pub;
    ros::Publisher est_vel_pub;
    
    EncoderListener listener = EncoderListener(n_);

    teleop_sub = n_.subscribe("motor_teleop/twist", 10, teleopCallback);
    obj_position_move_sub = n_.subscribe("/motorController/moveToObj", 1, objPositionMoveCallback);
    vel_left_pub = n_.advertise<std_msgs::Float32>("left_motor/cmd_vel", 10);
    vel_right_pub = n_.advertise<std_msgs::Float32>("right_motor/cmd_vel", 10);
    est_vel_pub = n_.advertise<geometry_msgs::Twist>("est_robot_vel/twist", 10);
    
    ros::Rate loop_rate(10);

    ros::NodeHandle nh("~");
	nh.getParam("alpha_left",alpha_left);
	nh.getParam("beta_left",beta_left);
	nh.getParam("alpha_right",alpha_right);
	nh.getParam("beta_right",beta_right);
	nh.getParam("beta_right",beta_right);
	nh.getParam("proportion_left",proportion_left);
    nh.getParam("proportion_right",proportion_right);

    double dt = 0.1;
    const double control_frequency = 124; //100; //10; //still of by about 4
    const double ticks_per_rev_left = 895.0;//3591.84;//900;
    const double ticks_per_rev_right = 900.0;
    const double base_ = 0.238;
    const double wheel_radius_ = 0.036;

    double int_err_left = 0.0;
    double int_err_right = 0.0;

    

/*
    //Controller parameters
    double alpha_left = 2.0;//7.5/5;//12
    double beta_left = 0.0;//30.5;//30.0/5;//20
    
    double alpha_right = 2.0;///5;//12
    double beta_right = 0.0;//30.0;///5;//20
    */
    while(ros::ok())
    {
		ROS_INFO("--------------------------------------------------------------");
        // Desired wheels angular velocities
		if (listener.last_reading_left.sec != 0 && listener.last_reading_left.nsec != 0 && listener.last_reading_right.sec != 0 && listener.last_reading_right.nsec != 0)
		{
            //ROS_INFO("proportion left : %f", proportion_left);
    		//ROS_INFO("proportion right : %f", proportion_right);

            float desired_wl = (lin_vel_ - base_*ang_vel_/2)/wheel_radius_;
            float desired_wr = (lin_vel_ + base_*ang_vel_/2)/wheel_radius_;
            ROS_INFO("Desired left : %f", desired_wl);
            ROS_INFO("Desired right : %f", desired_wr);
            // Code for calculating estimated current velocity based on encoders

	    	//int n=5, i, sum_left=0, sum_right=0;
            int last_change_left = (listener.last_change_left);
            int last_change_right = -(listener.last_change_right);

	    //double duration_left_sec = (double) listener.duration_left/1000000000;
	   // double duration_right_sec = (double) listener.duration_right/1000000000;

            //ROS_INFO("Encoder duration left : %lf", listener.duration_left);
            //ROS_INFO("Encoder duration right : %lf", listener.duration_right);

            //ROS_INFO("Encoder change left : %i", last_change_left);
            //ROS_INFO("Encoder change right : %i", last_change_right);

            float est_wl = last_change_left*2.0*PI*(1.0/listener.duration_left)/ticks_per_rev_left;
            float est_wr = last_change_right*2.0*PI*(1.0/listener.duration_right)/ticks_per_rev_right;

            ROS_INFO("Est left : %f", est_wl);
            ROS_INFO("Est right : %f", est_wr);

            float v = wheel_radius_*(est_wl + est_wr)/2.0;
            float omega = wheel_radius_*(est_wr-est_wl)/base_;

            ROS_INFO("current omega:%f, current v:%f", omega, v);

            //Controller

            float error_left = desired_wl - est_wl;
            float error_right = desired_wr - est_wr;

            ROS_INFO("Error left : %f", error_left);
            ROS_INFO("Error right : %f", error_right);

            //int_err_left += error_left*dt;
            int_err_left += error_left*listener.duration_left;
            //int_err_right += error_right*dt;
            int_err_right += error_right*listener.duration_right;

			//set max PWM values 100
            float pwm_left_float = std:: max(-100.0, (std::min(100.0, (proportion_left * desired_wl + alpha_left * error_left + beta_left * int_err_left))));
            float pwm_right_float =  std::max(-100.0, (std::min(100.0, (proportion_right * desired_wr + alpha_right * error_right + beta_right * int_err_right))));
    		if (abs(pwm_left_float)==100.0)
			{
				int_err_left -= error_left*listener.duration_left;
			}
			if  (abs(pwm_right_float)==100.0)
			{
				int_err_right -= error_right*listener.duration_right;
			}
            ROS_INFO("Integral error left: %f", int_err_left);
	    	ROS_INFO("Integral error right: %f", int_err_right);

            ROS_INFO("Estimated v: %f, Estimated omega: %f", v, omega);
            ROS_INFO("Wanted v: %f, Wanted omega: %f", lin_vel_, ang_vel_);

            //Publishing PWM to left and right motors
            std_msgs::Float32 msg1;
            std_msgs::Float32 msg2;
            
			if (lin_vel_ == 0 && ang_vel_ == 0)
			{
			    pwm_left_float = 0;
				msg1.data = pwm_left_float;
			    pwm_right_float = 0;
				msg2.data = pwm_right_float;
				int_err_left = 0;
				int_err_right = 0;
			}
		    else 
			{
				msg1.data = pwm_left_float;
		        msg2.data = pwm_right_float;
			}
			ROS_INFO ("PWM left: %f", pwm_left_float);
			ROS_INFO ("PWM right: %f", pwm_right_float);
            vel_left_pub.publish(msg1);
            vel_right_pub.publish(msg2);

            //Publishing estimated velosity of robot to calc odometry  
            
            geometry_msgs::Twist mes;

            mes.linear.x = v;
            mes.angular.z = omega;

            est_vel_pub.publish(mes);

            lin_vel_ = ang_vel_ = 0;
            listener.last_change_left = 0;
            listener.last_change_right = 0;
            listener.duration_left = 0;
            listener.duration_right = 0;
        }
		ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
