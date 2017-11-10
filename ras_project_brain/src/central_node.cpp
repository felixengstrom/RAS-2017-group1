
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>


class CentralNode
{
public:

    ros::NodeHandle n;
    ros::Publisher objectDetection_publisher;
    ros::Subscriber objectDetection_subscriber;

    CentralNode()
    {
        n = ros::NodeHandle("~");
        object_detected = 0;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 1);
        objectDetection_subscriber = n.subscribe("camera/image/object_flag", 1, &CentralNode::objectDetectionCallback, this);

    }

    ~CentralNode()
    {
        //delete CentralNode;
    }

    void objectDetectionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        object_detected = msg->data;
    }

    void pDetectionConvey()
    {
        std_msgs::String msg;

        if(1 == object_detected)
        {
             std::string detectionString("Object found");
             msg.data = detectionString;
             objectDetection_publisher.publish(msg);
             object_detected = 0;
        }

    }

private:
    bool object_detected;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_node");

  CentralNode central_node;

  /* Runs at a frequency of 10Hz */
  ros::Rate loop_rate(10);

  while (central_node.n.ok())
  {
    central_node.pDetectionConvey();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
