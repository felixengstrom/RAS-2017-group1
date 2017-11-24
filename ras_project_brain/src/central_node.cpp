
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

static bool object_detected_prev;
class CentralNode
{
public:

    ros::NodeHandle n;
    ros::Publisher objectDetection_publisher;
    ros::Publisher objectPosition_publisher;
    ros::Publisher motorStop_publisher;
    ros::Publisher arm_engageSuction_publisher;
    ros::Publisher objectCoordinateCalc_publisher;
    ros::Publisher tf_PickUp_publisher;
    ros::Subscriber objectDetection_subscriber;
    ros::Subscriber objectClass_subscriber;
    tf::TransformListener listener;

    CentralNode()
    {
        n = ros::NodeHandle("~");
        object_detected_current = 0;
        start_coordCalc = 0;
        object_x_position = 0.0;
        engage_suction = 0;
        stopMotor_toUpdate = 0;
        current_state = INITIAL_STATE;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 2);
        motorStop_publisher = n.advertise<std_msgs::Bool>("/pathFollow/start_stop", 1);
        objectPosition_publisher = n.advertise<std_msgs::Float32>("/motorController/moveToObj", 1);
        arm_engageSuction_publisher = n.advertise<std_msgs::Bool>("/uarm/engageSuction",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 1);
        tf_PickUp_publisher = n.advertise<std_msgs::Bool>("/tf/pickup_obj", 1);
        objectClass_subscriber = n.subscribe("/camera/object_class", 1, &CentralNode::objectClassCallback, this);
        objectDetection_subscriber = n.subscribe("/camera/object_detected", 1, &CentralNode::objectDetectionCallback, this);
    }

    ~CentralNode()
    {
        //delete CentralNode;
    }

    void objectDetectionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        object_detected_current = msg->data;
    }

    void objectClassCallback(const std_msgs::String::ConstPtr &msg)
    {
        classString.assign(msg->data);
    }

    void pObjDetectionStateMachine()
    {
        tf::StampedTransform transform;
        switch (current_state)
        {
        case INITIAL_STATE:
            if (1 == object_detected_current && (object_detected_prev!=object_detected_current))
            {
                ROS_INFO("In state object detected");
                current_state = OBJ_DETECTION_STATE;
                previous_state = INITIAL_STATE;
                /*Send "Object found" string to speaker*/
                std::string detectionString("Object found");
                msg_eSpeak_string.data = detectionString;
                objectDetection_publisher.publish(msg_eSpeak_string);
                msg_eSpeak_string.data = classString;
                objectDetection_publisher.publish(msg_eSpeak_string);
                /*Start the calculation of object coordinates wrt robot*/
                start_coordCalc = 1;
                stopMotor_toUpdate = 1;
                msg_tfObjCalc_bool.data = start_coordCalc;
                msg_motorStartStop_bool.data = stopMotor_toUpdate;
                objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
                motorStop_publisher.publish(msg_motorStartStop_bool);
                object_detected_prev = object_detected_current;
            }
            else if(0 == object_detected_current && (object_detected_prev!=object_detected_current))
            {
                //engage_suction = 0;
                object_detected_prev = object_detected_current;
                //msg_uarm_engageSuction_bool.data = engage_suction;
                //arm_engageSuction_publisher.publish(msg_uarm_engageSuction_bool);
            }
            break;
        case OBJ_DETECTION_STATE:
            if(INITIAL_STATE == previous_state)
            {
                ROS_INFO("In state move to object");
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
                  }while(diff_distance<-0.05 || diff_distance>0.05);
            }
            break;
        case MOVING_TO_POSITION_STATE:
            if(OBJ_DETECTION_STATE == previous_state)
            {
                ROS_INFO("In state object pickup");
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
                ROS_INFO("In state initial");
                current_state = INITIAL_STATE;
                previous_state = OBJ_PICKUP_STATE;
                start_coordCalc = 0;
                pickup_object = 0;
                stopMotor_toUpdate = 0;
                msg_motorStartStop_bool.data = stopMotor_toUpdate;
                msg_tfObjCalc_bool.data = start_coordCalc;
                tf_PickUp_publisher.publish(msg_tfPickUpObj_bool);
                objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
                motorStop_publisher.publish(msg_motorStartStop_bool);
            }
            break;
        }
    }

private:
    std::string classString;
    bool object_detected_current, start_coordCalc;
    bool stopMotor_toUpdate, engage_suction, pickup_object;
    int current_state, previous_state;
    float object_x_position, diff_distance;
    std_msgs::String msg_eSpeak_string;
    std_msgs::Bool msg_tfObjCalc_bool,msg_uarm_engageSuction_bool,msg_tfPickUpObj_bool, msg_motorStartStop_bool;
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
