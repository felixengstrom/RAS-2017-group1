#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
class CoordinateTransformer{


    tf::TransformListener listener;
	ros::NodeHandle node;
    ros::Subscriber sub;
	ros::Publisher uArmObj_position;

    public:
    void pickupCallback(const std_msgs::Int32::ConstPtr& msg);
    CoordinateTransformer(ros::NodeHandle& n);

};

CoordinateTransformer::CoordinateTransformer(ros::NodeHandle& n)
{
    node = n;
    sub = node.subscribe("pickup", 1, &CoordinateTransformer::pickupCallback, this);
	uArmObj_position =
	    node.advertise<geometry_msgs::Point>("uarm/moveToPose", 10);


}
void CoordinateTransformer::pickupCallback(const std_msgs::Int32::ConstPtr& msg){

    if (true){
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform("uarm_base", "object",ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
    }

        geometry_msgs::Point uArmObjPos_msg;
        uArmObjPos_msg.x = transform.getOrigin().x();
        uArmObjPos_msg.y = transform.getOrigin().y();
        uArmObjPos_msg.z = transform.getOrigin().z();
        uArmObj_position.publish(uArmObjPos_msg);
        ROS_INFO("should be sending");
    } else {
        geometry_msgs::Point uArmObjPos_msg;
        uArmObjPos_msg.x = 0.15;
        uArmObjPos_msg.y = 0;
        uArmObjPos_msg.z = 0.15;
    }

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_listener");
    ros::NodeHandle n;

    CoordinateTransformer tf(n);

    ros::Rate rate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
