
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <string>


class CentralNode
{
public:

    ros::NodeHandle n;
    ros::Publisher objectDetection_publisher;
    ros::Publisher objectPosition_publisher;
    ros::Subscriber objectDetection_subscriber;
    tf::TransformListener listener;

    CentralNode()
    {
        n = ros::NodeHandle("~");
        object_detected = 0;
        object_detected_flag = 1;
        start_coordCalc = 0;
        object_x_position = 0.0;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 1);
        objectPosition_publisher = n.advertise<std_msgs::Float32>("/motorController/moveToObj", 1);
        /*Needed only if x,y,z are being published randomly by the camera even when the object is not detected, else it can be removed
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 1);*/
        objectDetection_subscriber = n.subscribe("/camera/image/object_detected", 1, &CentralNode::objectDetectionCallback, this);
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
        /*Object_detected flag has been introduced to make sure that eSpeak is triggered
         * only once despite a periodic publish from the camera on object detection*/
        if(1 == object_detected && 1 == object_detected_flag)
        {
             /*Sending "Object found" string to speaker*/
             std::string detectionString("Object found");
             msg_eSpeak_string.data = detectionString;
             objectDetection_publisher.publish(msg_eSpeak_string);
             object_detected_flag = 0;
             /*Needed only if x,y,z are being published randomly by the camera even when the object is not detected, else it can be removed*/
             start_coordCalc = 1;
             /*msg_tfObjCalc_bool.data = start_coordCalc;
             objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);*/
        }
        else if(0 == object_detected)
        {
            start_coordCalc = 0;
            /*This flag might have to be updated after the object is picked up*/
            object_detected_flag = 1;
        }
    }

    /*Check if the object lies in the optimal position*/
    void pCoordCalculation()
    {
       tf::StampedTransform transform;
       ROS_INFO("%d",start_coordCalc);
       if(1 == start_coordCalc)
       {
          try
           {
               listener.waitForTransform("robot","object",ros::Time(0), ros::Duration(2));
               listener.lookupTransform("robot","object",ros::Time(0), transform);
           }
           catch(tf::TransformException &ex)
           {
               ROS_ERROR("%s",ex.what());
           }
           object_x_position = transform.getOrigin().x();
           /*******Convert into cm and put the exact values********/
           diff_distance = object_x_position - 25.0;
           ROS_INFO("Difference calculated --> %0.2f",diff_distance);
           if(diff_distance<-3 || diff_distance>3)
           {
               msg_odomDiffDist_float.data = diff_distance;
           }
           else
           {
               msg_odomDiffDist_float.data = 0.0;
           }
           objectPosition_publisher.publish(msg_odomDiffDist_float);
       }
    }

private:
    bool object_detected,object_detected_flag;
    bool start_coordCalc;
    float object_x_position, diff_distance;
    std_msgs::String msg_eSpeak_string;
    std_msgs::Bool msg_tfObjCalc_bool;
    std_msgs::Float32 msg_odomDiffDist_float;
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
    central_node.pCoordCalculation();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
