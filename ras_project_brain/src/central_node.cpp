
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <string>
#include <ras_project_brain/PickUpObj.h>

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
    ros::Publisher uarm_pickup_ack_publisher;
    ros::Publisher objectPickedUpSuccess_publisher;
    ros::Subscriber objectDetection_subscriber;
    ros::Subscriber objectClass_subscriber;

    ros::Subscriber uarmStart_subscriber;

    tf::TransformListener listener;
    ros::ServiceServer service;

    CentralNode()
    {
        n = ros::NodeHandle("~");

        startPickUp = 0;

        object_detected_current = 0;
        start_coordCalc = 0;
        object_x_position = 0.0;
        engage_suction = 0;
        stopMotor_toUpdate = 0;
        sm_complete = 0;
        obj_picked = 0;
        current_state = INITIAL_STATE;
        previous_state = OBJ_PICKUP_STATE;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 2);
        motorStop_publisher = n.advertise<std_msgs::Bool>("/robot/stop", 1);
        objectPosition_publisher = n.advertise<std_msgs::Float32>("/motorController/moveToObj", 1);
        arm_engageSuction_publisher = n.advertise<std_msgs::Bool>("/uarm/engageSuction",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 1);
        objectPickedUpSuccess_publisher = n.advertise<std_msgs::Bool>("/brain/pickupSuccess", 1);
        tf_PickUp_publisher = n.advertise<std_msgs::Bool>("/tf/pickup_obj", 1);
        objectClass_subscriber = n.subscribe("/camera/object_class", 1, &CentralNode::objectClassCallback, this);
        objectDetection_subscriber = n.subscribe("/camera/object_detected", 1, &CentralNode::objectDetectionCallback, this);
        //service = n.advertiseService("obj_pickup_sm", &CentralNode::pObjDetectionStateMachine, this);

        uarmStart_subscriber = n.subscribe("/brain/smStart", 1, &CentralNode::uarmStartPickupCallback, this);
        uarm_pickup_ack_publisher = n.advertise<std_msgs::Bool>("/brain/smEnd", 1);
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

    void uarmStartPickupCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        startPickUp = msg->data;
    }

    /*bool pObjDetectionStateMachine(ras_project_brain::PickUpObj::Request  &req, ras_project_brain::PickUpObj::Response &res)
    {*/
    bool pObjDetectionStateMachine()
    {
        //tf::StampedTransform transform;
        if (startPickUp == 1)
        {
            ros::Time begin = ros::Time::now();
            switch (current_state)
            {
            case INITIAL_STATE:
                if (INITIAL_STATE == current_state && OBJ_PICKUP_STATE == previous_state &&
                        1 == object_detected_current)
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
                }
                else
                {
                    ROS_INFO("Object not found here");
                    sm_complete = 1;
                    msg_smCompleteAck_bool.data = sm_complete;
                    uarm_pickup_ack_publisher.publish(msg_smCompleteAck_bool);
                }
                break;
            case OBJ_DETECTION_STATE:
                if(INITIAL_STATE == previous_state)
                {
                  ROS_INFO("In state move to object");
                  current_state = MOVING_TO_POSITION_STATE;
                  previous_state = OBJ_DETECTION_STATE;
                  ROS_INFO("%d",start_coordCalc);
                  /*This while loop is safe to run only when the object is in view*/
                  /*do{
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
                    }while(diff_distance<-0.05 || diff_distance>0.05);*/
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
                  /*wait approx for the pickup*/
                  while(ros::Time::now() - begin < ros::Duration(2.0));
                  /*Give one more try if failed*/
                  if (1 == object_detected_current)
                  {
                    ROS_INFO("Object pickup failed");
                  }
                  else
                  {
                      obj_picked = 1;
                      msg_objectPicked_custom.data = obj_picked;
                      objectPickedUpSuccess_publisher.publish(msg_objectPicked_custom);
                  }
                  /*while(count == 0)
                  {
                      if (1 == object_detected_current)
                      {
                          current_state = INITIAL_STATE;
                          previous_state = OBJ_PICKUP_STATE;
                      }
                      count++;
                  }*/
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
                  sm_complete = 1;
                  msg_smCompleteAck_bool.data = sm_complete;
                  uarm_pickup_ack_publisher.publish(msg_smCompleteAck_bool);
                }
                break;
            }
        //res.success = 1;
        }
        else
        {
            ROS_INFO("Waiting to pick object");
        }
        return true;
    }

private:
    std::string classString;
    bool object_detected_current, start_coordCalc, startPickUp, obj_picked;
    bool stopMotor_toUpdate, engage_suction, pickup_object, sm_complete;
    int current_state, previous_state;
    float object_x_position, diff_distance;
    std_msgs::String msg_eSpeak_string;
    std_msgs::Bool msg_tfObjCalc_bool,msg_uarm_engageSuction_bool,msg_tfPickUpObj_bool;
    std_msgs::Bool msg_motorStartStop_bool, msg_smCompleteAck_bool;
    std_msgs::Bool msg_objectPicked_custom;
    std_msgs::Float32 msg_odomDiffDist_float;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "central_node");

  CentralNode central_node;

  ros::Rate loop_rate(10);

  while (central_node.n.ok())
  {
    central_node.pObjDetectionStateMachine();
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}
