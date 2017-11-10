#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>

int main( int argc, char** argv)
{
    ros::init(argc, argv, "object_recognition");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1000, imageCallback);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
	ros::spinOnce();
    }
}
