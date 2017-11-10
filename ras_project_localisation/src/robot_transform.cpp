#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>


class RobotBroadcaster
{

    private:
         ros::NodeHandle n;
         ros::Subscriber sub;
         tf::Transform transform;
         tf::TransformBroadcaster br;

    public:
         RobotBroadcaster(ros::NodeHandle _n):n(_n), br(), transform()
         {
            sub = n.subscribe("/robot/pose", 10, &RobotBroadcaster::robotPoseCallback, this);
         
         };
        
         void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


};

void RobotBroadcaster::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    ROS_INFO("in callback");
    transform.setOrigin(tf::Vector3(x,y,0.0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map","robot"));
}

int main( int argc, char ** argv)
{
    ros::init(argc,argv,"robot_transform_broadcaster");
    ros::NodeHandle n;
    RobotBroadcaster rb(n);
    ros::spin();
}
