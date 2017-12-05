
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <ras_msgs/RAS_Evidence.h>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <ras_project_brain/PickUpObj.h>
#include <ras_project_brain/ObjPickup_Update.h>

#define INITIAL_STATE 0
#define OBJ_DETECTION_STATE 1
#define OBJ_UPDATION_STATE 2
#define START_EXIT_PREPARATION 3
#define GO_TO_OBJECT 4
#define PICKUP_OBJECT 5
#define GOTO_GOAL 6
#define GOAL_REACHED 7
#define DONE 8

struct object_details
{
    geometry_msgs::Vector3 location;
};

class FirstRunNode
{
public:

    ros::NodeHandle n;
    ros::Publisher objectDetection_publisher;
    ros::Publisher motorStop_publisher;
    ros::Publisher exploration_command_publisher;
    ros::Publisher objectCoordinateCalc_publisher;
    ros::Publisher robot_destination_publisher;
    ros::Publisher arm_engageSuction_publisher;
    ros::Publisher object_pickedup_publisher;
    ros::Publisher object_list_fileSave_publisher;

    ros::Publisher uarm_pickup_publisher;
    ros::Subscriber uarm_pickup_ack_subscriber;

    ros::Subscriber objectDetection_subscriber;
    ros::Subscriber objectClass_subscriber;
    ros::Subscriber objectPosition_subscriber;
    ros::Subscriber explorationCompletion_subscriber;
    ros::Subscriber robot_map_position_subscriber;
    ros::Subscriber robot_map_update_subscriber;
    ros::Subscriber object_pickup_success_subscriber;
    tf::TransformListener listener;
    ros::ServiceClient client;

    FirstRunNode()
    {
        n = ros::NodeHandle("~");
        begin = ros::Time::now();
        i = j = 0;
        exploration_start_stop = 0;
        exploration_completion = sm_complete_ack =0;
        item_num = 0;
        object_detected_current = object_detected_prev = 0;
        objectDistanceGoal_old = 200.0;
        start_coordCalc = 0;
        map_coord_updated = 0;
        pickup_success = 0;
        //start_pcl = 0;
        slowMotor_toUpdate = 0;
        uarm_smTrigger = 0;
        current_state = INITIAL_STATE;
        previous_state = OBJ_UPDATION_STATE;
        current_state_exit = START_EXIT_PREPARATION;
        previous_state_exit = GOAL_REACHED;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 2);
        motorStop_publisher = n.advertise<std_msgs::Bool>("/pathFollow/slowDown", 1);
        exploration_command_publisher = n.advertise<std_msgs::Bool>("/Exploration/Go",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 2);
        robot_destination_publisher = n.advertise<geometry_msgs::PoseStamped>("/robot/goal", 1);
        arm_engageSuction_publisher = n.advertise<std_msgs::Bool>("/uarm/engageSuction",1);
        object_pickedup_publisher = n.advertise<ras_project_brain::ObjPickup_Update>("/map/pickupSuccess",1);
        object_list_fileSave_publisher = n.advertise<std_msgs::Bool>("/object/listSave",1);

        uarm_pickup_publisher = n.advertise<std_msgs::Bool>("/brain/smStart", 1);
        uarm_pickup_ack_subscriber = n.subscribe("/brain/smEnd", 3, &FirstRunNode::uarmSMCompleteAckCallback, this);

        objectClass_subscriber = n.subscribe("/camera/object_class", 1, &FirstRunNode::objectClassCallback, this);
        objectDetection_subscriber = n.subscribe("/camera/object_detected", 1, &FirstRunNode::objectDetectionCallback, this);
        objectPosition_subscriber = n.subscribe("/evidence", 1, &FirstRunNode::objectPositionCallback, this);
        explorationCompletion_subscriber = n.subscribe("/Explored/done", 1, &FirstRunNode::explorationCompletionCallback, this);
        robot_map_position_subscriber = n.subscribe("/robot/pose", 1, &FirstRunNode::robotMapPositionCallback, this);
        robot_map_update_subscriber = n.subscribe("/map/coordUpdate", 1, &FirstRunNode::mapCoordUpdateCallback, this);
        object_pickup_success_subscriber = n. subscribe("/brain/pickupSuccess", 1, &FirstRunNode::objectPickupSuccessCallback, this);
        client = n.serviceClient<ras_project_brain::PickUpObj>("obj_pickup_sm");
    }

    ~FirstRunNode()
    {
        //delete FirstRunNode;
    }

    void objectDetectionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        object_detected_current = msg->data;
    }

    void objectClassCallback(const std_msgs::String::ConstPtr &msg)
    {
        classString.assign(msg->data);
    }

    void objectPositionCallback(const ras_msgs::RAS_Evidence::ConstPtr &msg)
    {
        list[i].location = msg->object_location.transform.translation;
    }

    void explorationCompletionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        exploration_completion = msg->data;
    }

    void robotMapPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        robotPosition = msg->pose.position;
    }

    void mapCoordUpdateCallback(const std_msgs::Bool::ConstPtr &msg)
    {
    	map_coord_updated = msg->data;
    }

    void uarmSMCompleteAckCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        sm_complete_ack = msg->data;
    }

    void objectPickupSuccessCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        pickup_success = msg->data;
    }

    void pExplorationStateMachine()
    {
        /*Slow the motors to update the position of the object/replan the path*/
        slowMotor_toUpdate = 1;
        msg_motorSlowDown_bool.data = slowMotor_toUpdate;
        motorStop_publisher.publish(msg_motorSlowDown_bool);

        if (ros::Time::now()- begin <= ros::Duration(3*60.0) && exploration_completion!=1 )
        {
            ROS_INFO("In state exploration");
            //continue exploration
            exploration_start_stop = 1;
            msg_exploration_startStop_bool.data = exploration_start_stop;
            exploration_command_publisher.publish(msg_exploration_startStop_bool);
            switch (current_state)
            {
            case INITIAL_STATE:
                if (1 == object_detected_current && OBJ_UPDATION_STATE == previous_state &&
                        (object_detected_prev != object_detected_current))
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
                    msg_tfObjCalc_bool.data = start_coordCalc;
                    objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);

                    object_detected_prev = object_detected_current;
                }
                else if(0 == object_detected_current && (object_detected_prev!=object_detected_current))
                {
                    object_detected_prev = object_detected_current;
                }
                break;
            case OBJ_DETECTION_STATE:
                if (INITIAL_STATE == previous_state && 1 == map_coord_updated)
                {
                    ROS_INFO("In state object updated");
                    current_state = OBJ_UPDATION_STATE;
                    previous_state = OBJ_DETECTION_STATE;
                    /*Update the object in the brain node's database*/
                    ROS_INFO("Position found: x=%.2f, y=%.2f", list[i].location.x,list[i].location.y);
                    j = 0;
                    i += 1;
                    /*Stop the calculation of object coordinates wrt robot*/
                    start_coordCalc = 0;
                    msg_tfObjCalc_bool.data = start_coordCalc;
                    objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
                }
                break;
            case OBJ_UPDATION_STATE:
                if (OBJ_DETECTION_STATE == previous_state)
                {
                    ROS_INFO("In state initial");
                    current_state = INITIAL_STATE;
                    previous_state = OBJ_UPDATION_STATE;
                }
                break;
            }

        }
        else
        {
            ROS_INFO("In state return to goal");
            /* Stop exploration */
            exploration_start_stop = 0;
            msg_exploration_startStop_bool.data = exploration_start_stop;
            exploration_command_publisher.publish(msg_exploration_startStop_bool);
            float distance_robObj;
            std::string exitString("Going home");
            switch(current_state_exit)
            {
                case START_EXIT_PREPARATION:
                    /* Find the object closer to the goal */
                	ROS_INFO("Start exit preparation");
                    current_state_exit = GO_TO_OBJECT;
                    for (int obj=0;obj<i;obj++)
                    {
                        objectDistanceGoal_new = sqrt(pow(list[obj].location.x,2)+
                                                  pow(list[obj].location.y,2));
                        if (objectDistanceGoal_new <= objectDistanceGoal_old)
                        {
                            objectDistanceGoal_old = objectDistanceGoal_new;
                            item_num = obj;
                        }
                    }
                    msg_robotDestination.header.frame_id = "map";
                    msg_robotDestination.header.stamp = ros::Time::now();
                    msg_robotDestination.pose.position.x = list[item_num].location.x;
                    msg_robotDestination.pose.position.y = list[item_num].location.y;
                    msg_robotDestination.pose.position.z = 0.0;
                    robot_destination_publisher.publish(msg_robotDestination);
                    msg_eSpeak_string.data = exitString;
                    objectDetection_publisher.publish(msg_eSpeak_string);
                break;
                case GO_TO_OBJECT:
                    current_state_exit = PICKUP_OBJECT;
                    distance_robObj = sqrt(pow((robotPosition.x-list[item_num].location.x),2)+
                            pow((robotPosition.y-list[item_num].location.y),2));
                        ROS_INFO("Distance to the object to be picked: %0.2f", distance_robObj);
                    /*if the position is not reached*/
                    if(distance_robObj > 0.3)
                    {
                        current_state_exit = GO_TO_OBJECT;
                    }
                break;
                case PICKUP_OBJECT:
                    current_state_exit = GOTO_GOAL;
                    /*Start uarm_SM*/
                    uarm_smTrigger = 1;
                    msg_uarm_pickup_bool.data = uarm_smTrigger;
                    uarm_pickup_publisher.publish(msg_uarm_pickup_bool);
                    if(sm_complete_ack!=1)
                    {
                        current_state_exit = PICKUP_OBJECT;
                    }
                    else
                    {
                        if(1 == pickup_success)
                        {
                            msg_pickup_update_custom.coord.x = list[item_num].location.x;
                            msg_pickup_update_custom.coord.y = list[item_num].location.y;
                            msg_pickup_update_custom.coord.z = list[item_num].location.z;
                            msg_pickup_update_custom.pickedUp = 1;
                            object_pickedup_publisher.publish(msg_pickup_update_custom);
                        }
                    }
                    /*srv.request.objPickUpState = 1;
                    if (client.call(srv))
                    {
                        ROS_INFO("Object Pickup call success");
                    }
                    else
                    {
                    	ROS_INFO("Warning: Pickup call failed!!");
                    }*/
                break;
                case GOTO_GOAL:
                    ROS_INFO("Sent the goal position");
                    /*Stop the object pickup command*/
                    uarm_smTrigger = 0;
                    msg_uarm_pickup_bool.data = uarm_smTrigger;
                    uarm_pickup_publisher.publish(msg_uarm_pickup_bool);
                	current_state_exit = GOAL_REACHED;
                    msg_robotDestination.header.frame_id = "map";
                    msg_robotDestination.header.stamp = ros::Time::now();
                    msg_robotDestination.pose.position.x = 0.22;
                    msg_robotDestination.pose.position.y = 0.22;
                    msg_robotDestination.pose.position.z = 0.0;
                    robot_destination_publisher.publish(msg_robotDestination);
                    /*reset the map update*/
                    msg_pickup_update_custom.pickedUp = 0;
                    object_pickedup_publisher.publish(msg_pickup_update_custom);
                break;
                case GOAL_REACHED:
                    ROS_INFO("Going to the goal position");
                    current_state_exit = DONE;
                    /*wait till the goal is reached*/
                    if (0.15 <= robotPosition.x && 0.15 <= robotPosition.y)
                    {
                        current_state_exit = GOAL_REACHED;
                    }
                break;
                case DONE:
                    ROS_INFO("Completed!!");
                    current_state_exit = 15;
                    /*drop the object: service call to uArm*/
                    /*disengage suction*/
                    engage_suction = 0;
                    msg_uarm_engageSuction_bool.data = engage_suction;
                    arm_engageSuction_publisher.publish(msg_uarm_engageSuction_bool);
                    /*Request to save the object list in fa file*/
                    msg_objectFileSave_bool.data = 1;
                    object_list_fileSave_publisher.publish(msg_objectFileSave_bool);
                break;
                default:
                    ROS_INFO("Task over");
            }
        }
    }

private:
    std::string classString;
    bool object_detected_current, start_coordCalc, start_pcl, object_detected_prev, map_coord_updated;
    bool slowMotor_toUpdate, exploration_start_stop, pickup_object,exploration_completion;
    bool uarm_smTrigger, sm_complete_ack, engage_suction, pickup_success;
    int current_state, previous_state;
    int current_state_exit, previous_state_exit;
    int item_num;
    float objectDistanceGoal_new,objectDistanceGoal_old;
    std_msgs::String msg_eSpeak_string;
    geometry_msgs::PoseStamped msg_robotDestination;
    geometry_msgs::Point robotPosition;
    std_msgs::Bool msg_tfObjCalc_bool,msg_exploration_startStop_bool,msg_cameraPcl_startStop_bool;
    std_msgs::Bool msg_motorSlowDown_bool, msg_uarm_pickup_bool, msg_uarm_engageSuction_bool;
    std_msgs::Bool msg_objectFileSave_bool;
    ras_project_brain::ObjPickup_Update msg_pickup_update_custom;
    std_msgs::Float32 msg_odomDiffDist_float;
    ros::Time begin;
    object_details list[10];
    int i,j;
    ras_project_brain::PickUpObj srv;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "firstRun_node");

  FirstRunNode first_run_node;

  /* Runs at a frequency of 10Hz */
  ros::Rate loop_rate(10);

  while (first_run_node.n.ok())
  {
    first_run_node.pExplorationStateMachine();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

