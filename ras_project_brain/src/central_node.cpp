
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <string>

#define INITIAL_STATE 0
#define OBJ_DETECTION_STATE 1
#define MOVING_TO_POSITION_STATE 2
#define OBJ_PICKUP_STATE 3


class CentralNode
{
public:

    ros::NodeHandle n;
    ros::Publisher objectDetection_publisher;
    ros::Publisher objectPosition_publisher;
    ros::Publisher arm_engageSuction_publisher;
    ros::Publisher objectCoordinateCalc_publisher;
    ros::Publisher tf_PickUp_publisher;
    ros::Subscriber objectDetection_subscriber;
    tf::TransformListener listener;

    CentralNode()
    {
        n = ros::NodeHandle("~");
        object_detected = 0;
        object_detected_flag = 1;
        start_coordCalc = 0;
        object_x_position = 0.0;
        engage_suction = 0;
        current_state = INITIAL_STATE;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 1);
        objectPosition_publisher = n.advertise<std_msgs::Float32>("/motorController/moveToObj", 1);
        arm_engageSuction_publisher = n.advertise<std_msgs::Bool>("uarm/engageSuction",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 1);
        tf_PickUp_publisher = n.advertise<std_msgs::Bool>("/tf/pickup_obj", 1);
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

    void pObjDetectionStateMachine()
    {
        tf::StampedTransform transform;
        switch (current_state)
        {
        case INITIAL_STATE:
            if (1 == object_detected)
            {
                current_state = OBJ_DETECTION_STATE;
                previous_state = INITIAL_STATE;
                /*Send "Object found" string to speaker*/
                if (1 == object_detected_flag)
                {
                    std::string detectionString("Object found");
                    msg_eSpeak_string.data = detectionString;
                    objectDetection_publisher.publish(msg_eSpeak_string);
                    object_detected_flag = 0;
                }
                /*Start the calculation of object coordinates wrt robot*/
                start_coordCalc = 1;
                msg_tfObjCalc_bool.data = start_coordCalc;
                objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
            }
            break;
        case OBJ_DETECTION_STATE:
            if(INITIAL_STATE == previous_state)
            {
                current_state = MOVING_TO_POSITION_STATE;
                previous_state = OBJ_DETECTION_STATE;
                ROS_INFO("%d",start_coordCalc);
                do{
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
                    diff_distance = object_x_position - 0.26;
                    ROS_INFO("Difference calculated --> %0.2f",diff_distance);
                    if (diff_distance<-0.03 || diff_distance>0.03)
                    {
                        msg_odomDiffDist_float.data = diff_distance;
                    }
                    else
                    {
                        msg_odomDiffDist_float.data = 0.0;
                    }
                    objectPosition_publisher.publish(msg_odomDiffDist_float);
                  }while(diff_distance<-0.03 || diff_distance>0.03);
            }
            break;
        case MOVING_TO_POSITION_STATE:
            if(OBJ_DETECTION_STATE == previous_state)
            {
                current_state = OBJ_PICKUP_STATE;
                previous_state = MOVING_TO_POSITION_STATE;
                engage_suction = 1;
                pickup_object = 1;
                /*arm related messages are published here
                 * 1. To start suction
                 * 2. To request the arm to move to the desired x,y,z coordinate*/
                msg_uarm_engageSuction_bool.data = engage_suction;
                msg_tfPickUpObj_bool.data = pickup_object;
                tf_PickUp_publisher.publish(msg_tfPickUpObj_bool);
                arm_engageSuction_publisher.publish(msg_uarm_engageSuction_bool);
            }
            break;
        case OBJ_PICKUP_STATE:
            if (MOVING_TO_POSITION_STATE == previous_state)
            {
                current_state = INITIAL_STATE;
                previous_state = OBJ_PICKUP_STATE;
                engage_suction = 0;
                start_coordCalc = 0;
                pickup_object = 0;
                object_detected_flag = 1;
                msg_tfObjCalc_bool.data = start_coordCalc;
                msg_uarm_engageSuction_bool.data = engage_suction;
                tf_PickUp_publisher.publish(msg_tfPickUpObj_bool);
                arm_engageSuction_publisher.publish(msg_uarm_engageSuction_bool);
                objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
            }
            break;
        }
    }

private:
    bool object_detected,object_detected_flag;
    bool start_coordCalc, engage_suction, pickup_object;
    int current_state, previous_state;
    float object_x_position, diff_distance;
    std_msgs::String msg_eSpeak_string;
    std_msgs::Bool msg_tfObjCalc_bool,msg_uarm_engageSuction_bool,msg_tfPickUpObj_bool;
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
    central_node.pObjDetectionStateMachine();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
