
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "phidgets/motor_encoder.h"
#include <geometry_msgs/Twist.h>

#define _USE_MATH_DEFINES
#include <cmath>

double lin_vel_;
double ang_vel_;
int delta_encoder1;
int delta_encoder2;
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
    ROS_INFO("desired_w: %f", msg->linear.x);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n_;
    //ros::Subscriber twist_sub;
    ros::Subscriber teleop_sub;
    ros::Publisher velR_pub ;
    ros::Publisher velL_pub ;
    
    EncoderListener listener = EncoderListener(n_);

    //encoderR_sub = n_.subscribe("right_motor/encoder", 1000, encoderRCallback);
    //encoderL_sub = n_.subscribe("left_motor/encoder", 1000, encoderLCallback);
    teleop_sub = n_.subscribe("motor_teleop/twist", 10, teleopCallback);
    velR_pub = n_.advertise<std_msgs::Float32>("right_motor/cmd_vel", 10);

    velL_pub = n_.advertise<std_msgs::Float32>("left_motor/cmd_vel", 10);

    //create node

    ros::Rate loop_rate(10);

    double dt = 0.01;
    const double control_frequency = 10;
    const double ticks_per_rev = 360;
    const double base_ = 0.238;
    const double wheel_radius_ = 0.036;


    while(ros::ok())
    {

        // Desired wheel angular velocitis
            double desired_wL = (lin_vel_ - base_*ang_vel_/2)/wheel_radius_;
            double desired_wR = (lin_vel_ + base_*ang_vel_/2)/wheel_radius_;
   
            // Code for calculating estimated current velocity based on encoders
            int last_change_left = listener.last_change_left;
            int last_change_right = listener.last_change_right;

            float est_wl = last_change_left*2*PI*control_frequency/ticks_per_rev;
            float est_wr = last_change_right*2*PI*control_frequency/ticks_per_rev;

            float v = wheel_radius_*(est_wl + est_wr)/2;
            float omega = wheel_radius_*(est_wr-est_wl)/2;

            ROS_INFO("current omega:%f, current v:%f", omega, v); 

            //Controller
        	//double  delta_encoder_double = double (delta_encoder);
    
            //double estimated_w = (delta_encoder_double*2.0*M_PI*control_frequency)/(ticks_per_rev);
            
            //ROS_INFO("estimated_w: %f", estimated_w);

            //double error = desired_w - estimated_w;
            //ROS_INFO("desired_w: %f", desired_w);
 
            //int_err += error*dt;
            
            //double val  = *point_int_err;
            //double pwm_double = alpha*error+beta*int_err;
            //int pwm = (int)(pwm_double);
            float pwmL = (float) desired_wL * 2;
            float pwmR = (float) desired_wR * 2;
            //ROS_INFO("R: %f", pwmR);
            //ROS_INFO("L: %f", pwmL);
            //Create msg
            std_msgs::Float32 msg;
            msg.data = pwmR;
     
            //Publish msg
            velR_pub.publish(msg);


            msg.data = pwmL;
     
            //Publish msg
            velL_pub.publish(msg);
            lin_vel_ = ang_vel_ = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
