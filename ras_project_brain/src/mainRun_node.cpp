/********Main Run Node***********/
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <ras_project_brain/PickUpObj.h>

#define PRE_INITIAL_STATE 0
#define PRE_INITIAL_PREP_STATE 1
#define INITIAL_STATE 2
#define OBJECT_POSITION_SEND 3
#define MOVE_TO_POSITION 4
#define OBJ_DETECTION_STATE 5
#define OBJ_PICKUP_STATE 6
#define START_EXIT_PREPARATION 7
#define GO_TO_OBJECT 8
#define PICKUP_OBJECT 9
#define GOTO_GOAL 10
#define GOAL_REACHED 11
#define DONE 12

struct object_details
{
    geometry_msgs::Point location;
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
    ros::Publisher object_listLoad_publisher;
    ros::Publisher robot_stop_publisher;

    ros::Publisher uarm_pickup_publisher;
    ros::Subscriber uarm_pickup_ack_subscriber;

    ros::Subscriber objectDetection_subscriber;
    ros::Subscriber objectClass_subscriber;
    ros::Subscriber explorationCompletion_subscriber;
    ros::Subscriber robot_map_position_subscriber;
    ros::Subscriber object_number_update;
    ros::Subscriber objectPosition_subscriber;
    tf::TransformListener listener;
    ros::ServiceClient client;

    FirstRunNode()
    {
        n = ros::NodeHandle("~");
        begin = ros::Time::now();
        exploration_start_stop = 0;
        exploration_completion = sm_complete_ack =0;
        item_num = 0;
        object_detected_current = object_detected_prev = 0;
        objectDistanceGoal_old = 200.0;
        start_coordCalc = 0;
        index = no = count = 0;
        startup_flag = 0;
        slowMotor_toUpdate = goto_goal_flag = 0;
        pickup_complete = 0;
        uarm_smTrigger = 0;
        stop_robot = 0;
        current_state = PRE_INITIAL_STATE;
        current_state_exit = GOTO_GOAL;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 2);
        motorStop_publisher = n.advertise<std_msgs::Bool>("/pathFollow/slowDown", 1);
        exploration_command_publisher = n.advertise<std_msgs::Bool>("/Exploration/Go",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 2);
        robot_destination_publisher = n.advertise<geometry_msgs::PoseStamped>("/robot/goal", 1);
        arm_engageSuction_publisher = n.advertise<std_msgs::Bool>("/uarm/engageSuction",1);
        object_listLoad_publisher = n.advertise<std_msgs::Bool>("/uarm/engageSuction",1);
        robot_stop_publisher = n.advertise<std_msgs::Bool>("/robot/stop",1);

        uarm_pickup_publisher = n.advertise<std_msgs::Bool>("/brain/smStart", 1);
        uarm_pickup_ack_subscriber = n.subscribe("/brain/smEnd", 3, &FirstRunNode::uarmSMCompleteAckCallback, this);

        objectClass_subscriber = n.subscribe("/camera/object_class", 1, &FirstRunNode::objectClassCallback, this);
        objectDetection_subscriber = n.subscribe("/camera/object_detected", 1, &FirstRunNode::objectDetectionCallback, this);
        explorationCompletion_subscriber = n.subscribe("/Explored/done", 1, &FirstRunNode::explorationCompletionCallback, this);
        objectPosition_subscriber = n.subscribe("/object/coordinates", 20, &FirstRunNode::objectPositionCallback, this);
        robot_map_position_subscriber = n.subscribe("/robot/pose", 1, &FirstRunNode::robotMapPositionCallback, this);
        object_number_update = n.subscribe("/object/numberLoad", 1, &FirstRunNode::objNumUpdateCallback, this);
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

    void objectPositionCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        list[index].location.x = msg->x;
        list[index].location.y = msg->y;
        list[index].location.z = msg->z;
        index++;
    }

    void explorationCompletionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        exploration_completion = msg->data;
    }

    void robotMapPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        robotPosition = msg->pose.position;
    }

    void objNumUpdateCallback(const std_msgs::Int16::ConstPtr &msg)
    {
        item_num = msg->data;
    }

    void uarmSMCompleteAckCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        sm_complete_ack = msg->data;
    }

    void pExplorationStateMachine()
    {
        /*stop the exploration node*/
        exploration_start_stop = 0;
        msg_exploration_startStop_bool.data = exploration_start_stop;
        exploration_command_publisher.publish(msg_exploration_startStop_bool);
        /*Continue to run the robot slowly*/
        slowMotor_toUpdate = 1;
        msg_motorSlowDown_bool.data = slowMotor_toUpdate;
        motorStop_publisher.publish(msg_motorSlowDown_bool);

        if (ros::Time::now() - begin <= ros::Duration(180.0) && 0 == goto_goal_flag
                && pickup_complete != 1)
        {
            int flag = 1;
            int temp;
            ROS_INFO("In state ready to pick objects");
            switch (current_state)
            {
            case PRE_INITIAL_STATE:
                current_state = PRE_INITIAL_PREP_STATE;
                obj_startupLoad_bool.data = 1;
                object_listLoad_publisher.publish(obj_startupLoad_bool);
                break;
            case PRE_INITIAL_PREP_STATE:
                current_state = INITIAL_STATE;
                obj_startupLoad_bool.data = 0;
                object_listLoad_publisher.publish(obj_startupLoad_bool);

                if(count != item_num)
                {
                    count++;
                    current_state = PRE_INITIAL_PREP_STATE;
                }
                else
                {
                    /*Calculate the positions in descending order*/
                    for(int i = 1; (i <= item_num) && flag; i++)
                    {
                        flag = 0;
                        for (int j=0; j < (item_num -1); j++)
                        {
                            if (list[j+1].location.x > list[j].location.x)
                            {
                             temp = list[j].location.x;
                             list[j].location.x = list[j+1].location.x;
                             list[j+1].location.x = temp;
                             flag = 1;
                            }
                        }
                    }
                }

                break;
            case INITIAL_STATE:
                current_state = OBJECT_POSITION_SEND;
                /*Send the goal position to robot
                 * If end of object list reached ->
                 * go to default state to prepare to exit*/
                if(item_num!=no)
                {
                    msg_robotDestination.header.frame_id = "map";
                    msg_robotDestination.header.stamp = ros::Time::now();
                    msg_robotDestination.pose.position.x = list[no].location.x;
                    msg_robotDestination.pose.position.y = list[no].location.y;
                    msg_robotDestination.pose.position.z = 0.0;
                    robot_destination_publisher.publish(msg_robotDestination);
                    no += 1;
                }
                else
                {
                    pickup_complete = 1;
                }
                break;
             case OBJECT_POSITION_SEND:
                current_state = MOVE_TO_POSITION;
                /*Check if the goal is reached*/
                if (robotPosition.x <= (list[no].location.x - 0.3) &&
                        robotPosition.y <= (list[no].location.y- 0.3))
                {
                    current_state = OBJECT_POSITION_SEND;
                }
                break;
            case MOVE_TO_POSITION:
                current_state = OBJ_DETECTION_STATE;
                if ((1 == object_detected_current)&& (object_detected_prev != object_detected_current))
                {
                    ROS_INFO("In state moved to position");
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
                else
                {
                    std::string detectionString("Oops object not found");
                    msg_eSpeak_string.data = detectionString;
                    objectDetection_publisher.publish(msg_eSpeak_string);
                    current_state = INITIAL_STATE;
                }
                break;
            case OBJ_DETECTION_STATE:
                current_state = OBJ_PICKUP_STATE;
                stop_robot = 1;
                msg_stopRobot_bool.data = stop_robot;
                robot_stop_publisher.publish(msg_stopRobot_bool);
                uarm_smTrigger = 1;
                msg_uarm_pickup_bool.data = uarm_smTrigger;
                uarm_pickup_publisher.publish(msg_uarm_pickup_bool);
                if(sm_complete_ack!=1)
                {
                    current_state = OBJ_DETECTION_STATE;
                }
                break;
            case OBJ_PICKUP_STATE:
                ROS_INFO("In state initial");
                current_state = INITIAL_STATE;
                stop_robot = 0;
                msg_stopRobot_bool.data = stop_robot;
                robot_stop_publisher.publish(msg_stopRobot_bool);
                /*Stop the object pickup command*/
                uarm_smTrigger = 0;
                msg_uarm_pickup_bool.data = uarm_smTrigger;
                uarm_pickup_publisher.publish(msg_uarm_pickup_bool);
                start_coordCalc = 0;
                msg_tfObjCalc_bool.data = start_coordCalc;
                objectCoordinateCalc_publisher.publish(msg_tfObjCalc_bool);
                /*Go to goal*/
                goto_goal_flag = 1;
                break;
            default:
                ;
            }

        }
        else
        {
            ROS_INFO("In state return to goal");
            switch(current_state_exit)
            {
                case GOTO_GOAL:
                    ROS_INFO("Sent the goal position");
                	current_state_exit = GOAL_REACHED;
                    msg_robotDestination.header.frame_id = "map";
                    msg_robotDestination.header.stamp = ros::Time::now();
                    msg_robotDestination.pose.position.x = 0.22;
                    msg_robotDestination.pose.position.y = 0.22;
                    msg_robotDestination.pose.position.z = 0.0;
                    robot_destination_publisher.publish(msg_robotDestination);
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
                    if (0 == pickup_complete)
                        current_state_exit = GOTO_GOAL;
                    else
                        current_state_exit = 16;
                    goto_goal_flag = 0;
                    /*drop the object: service call to uArm*/
                    /*disengage suction*/
                    engage_suction = 0;
                    msg_uarm_engageSuction_bool.data = engage_suction;
                    arm_engageSuction_publisher.publish(msg_uarm_engageSuction_bool);
                break;
                default:
                    stop_robot = 1;
                    msg_stopRobot_bool.data = stop_robot;
                    robot_stop_publisher.publish(msg_stopRobot_bool);
                    ROS_INFO("Task over");
            }
        }
    }

private:
    std::string classString;
    bool object_detected_current, start_coordCalc, object_detected_prev, stop_robot;
    bool slowMotor_toUpdate, exploration_start_stop, pickup_object,exploration_completion;
    bool uarm_smTrigger, sm_complete_ack, engage_suction, goto_goal_flag, pickup_complete;
    bool startup_flag;
    int current_state, current_state_exit;
    int item_num;
    float objectDistanceGoal_new,objectDistanceGoal_old;
    std_msgs::String msg_eSpeak_string;
    geometry_msgs::PoseStamped msg_robotDestination;
    geometry_msgs::Point robotPosition;
    std_msgs::Bool msg_tfObjCalc_bool,msg_exploration_startStop_bool,obj_startupLoad_bool;
    std_msgs::Bool msg_motorSlowDown_bool, msg_uarm_pickup_bool, msg_uarm_engageSuction_bool;
    std_msgs::Bool msg_stopRobot_bool;
    std_msgs::Float32 msg_odomDiffDist_float;
    ros::Time begin;
    object_details list[40];
    int index, no, count;
    ras_project_brain::PickUpObj srv;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mainRun_node");

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

