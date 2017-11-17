#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <phidgets/motor_encoder.h>

const static float PI = acos(-1);
class OdometryNode
{
    private:
        ros::NodeHandle n_;
        ros::Publisher odom_pub; 
        ros::Subscriber sub;
        ros::Subscriber imu;
        tf::Transform transform;
        tf::TransformBroadcaster br;
        tf::Quaternion q;
        ros::Time time;
        ros::Time time_imu;
        double current_v, current_w;
        double current_x, current_y, current_omega;
	    float speed_x, speed_y, speed_omega;
        float imu_x, imu_y, imu_omega;
	    float offset_x, offset_y;

    public:
        OdometryNode() 
        {
            time = ros::Time::now();
            current_x = current_y = 0;
            current_v = current_w = 0;
	        current_omega = PI/2;
            imu_x = imu_y = imu_omega = 0;
	        offset_x = offset_y = 0;
            n_ = ros::NodeHandle();
            sub = n_.subscribe("est_robot_vel/twist", 10, &OdometryNode::VelocityCallback, this);
            imu = n_.subscribe("imu/data", 10, &OdometryNode::IMUCallback, this);
        }
        void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg );
        void UpdatePosition();
        void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

void OdometryNode::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_v = msg->linear.x;
    current_w = msg->angular.z;
}

void OdometryNode::UpdatePosition()
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - time).toSec();
    time = current_time;
    current_x = current_x + cos(current_omega)*current_v*dt;
    current_y = current_y + sin(current_omega)*current_v*dt;
    current_omega = current_omega + current_w*dt;
    if (current_omega > PI) {
	current_omega = fmod(current_omega,PI)-PI;
    }
    if (current_omega < -PI)
	current_omega = fmod(current_omega,PI)+PI;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = current_x;
    pose.pose.position.y = current_y;
    pose.pose.orientation.w = cos(0.5*current_omega);
    pose.pose.orientation.z = sin(0.5*current_omega);

    transform.setOrigin( tf::Vector3(current_x, current_y, 0.0) );
    q.setRPY(0, 0, current_omega);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
}

void OdometryNode::IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (offset_x == 0)
    {
	time_imu = ros::Time::now();
	offset_x = msg->linear_acceleration.x;
	offset_y = msg->linear_acceleration.y;
    }
    else
    {
    	ros::Time current_time = ros::Time::now();
    	double dt = (current_time - time_imu).toSec();
    	time_imu = current_time;
    	imu_x += speed_x*dt;
    	imu_y += speed_y*dt;
	if (std::abs(msg->linear_acceleration.x - offset_x) >= 0.5)
    	    speed_x += (msg->linear_acceleration.x - offset_x)*dt;
    	if (std::abs(msg->linear_acceleration.y - offset_y) >= 0.5)
	    speed_y += (msg->linear_acceleration.y - offset_y)*dt;
    	imu_omega += msg->angular_velocity.z*dt;
    	if (imu_omega > PI)
    	    imu_omega = fmod(imu_omega, PI)-PI;
    	else if (imu_omega < -PI)
	    imu_omega = PI+fmod(imu_omega, PI);
//	ROS_INFO("DT : %lf", dt);
//   	ROS_INFO("ACC_X: %f, ACC_Y: %f, ACC_OMEGA: %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->angular_velocity.z);
//    	ROS_INFO("SPEED_X: %f, SPEED_Y: %f, SPEED_OMEGA: %f", speed_x, speed_y, speed_omega);
//    	ROS_INFO("IMU_x: %f, IMU_y: %f, IMU_omega: %f", imu_x, imu_y, imu_omega);
    }
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "odometry");
    OdometryNode on  = OdometryNode();
    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        on.UpdatePosition();
        rate.sleep();
    }
    return 0;
}
