#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>


class RobotBroadcaster
{

    private:
         ros::NodeHandle n;
         ros::Subscriber sub;
         ros::Publisher pub;
         tf::Transform transform;
         tf::TransformListener listener;
         geometry_msgs::PoseStamped lastPose;
         tf::TransformBroadcaster br;

    public:
         RobotBroadcaster(ros::NodeHandle _n):n(_n), br(), transform()
         {
            sub = n.subscribe("/localisation/pose", 10,
                              &RobotBroadcaster::robotPoseCallback, this);
            pub = n.advertise<geometry_msgs::PoseStamped>("/robot/pose", 10);
         
         };
         void publishCurrentPose();
        
         void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


};
void RobotBroadcaster::publishCurrentPose()
{
    geometry_msgs::PoseStamped currentPose(lastPose);
    tf::StampedTransform transform;
    ros::Time t  = ros::Time::now();
    try{
        listener.waitForTransform("odom",lastPose.header.stamp,
                                  "odom",t ,"map",ros::Duration(1));
        listener.lookupTransform( "odom",lastPose.header.stamp,
                                  "odom",t ,"map", transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("particle update failed");
        std::cout << ex.what() << "\n";
    }

    double currentAng = atan2(lastPose.pose.orientation.z,
                              lastPose.pose.orientation.w)*2;

    currentPose.pose.position.x += cos(currentAng)*transform.getOrigin().x()
                                  -sin(currentAng)*transform.getOrigin().y();
    currentPose.pose.position.y += cos(currentAng)*transform.getOrigin().y()
                                 +sin(currentAng)*transform.getOrigin().x();

    double changeAng = tf::getYaw(transform.getRotation());

    currentPose.pose.orientation.w = cos((currentAng + changeAng )*0.5);
    currentPose.pose.orientation.z = sin((currentAng + changeAng )*0.5);
    currentPose.header.stamp = t;
    pub.publish(currentPose);
    double x = currentPose.pose.position.x;
    double y = currentPose.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(currentPose.pose.orientation, q);
    transform.setOrigin(tf::Vector3(x,y,0.0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"map","robot"));

}

void RobotBroadcaster::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    lastPose = *msg;
    ROS_INFO("callback");
}

int main( int argc, char ** argv)
{
    ros::init(argc,argv,"robot_transform_broadcaster");
    ros::NodeHandle n;
    RobotBroadcaster rb(n);

    ros::Rate rate(100);
    while (ros::ok())
    {
        rb.publishCurrentPose();
        ros::spinOnce();
        rate.sleep();
    }
}
