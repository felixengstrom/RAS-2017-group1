
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <ras_project_brain/SetGoalPoint.h>

#define INITIAL_STATE 0
#define OBJ_DETECTION_STATE 1
#define OBJ_UPDATION_STATE 2
#define START_EXIT_PREPARATION 3
#define GO_TO_OBJECT 3
#define REACH_GOAL 4

struct object_details
{
    geometry_msgs::Point location;
    std::string colour_type[3];
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
    ros::Publisher cameraPcl_publisher;
    ros::Subscriber objectDetection_subscriber;
    ros::Subscriber objectClass_subscriber;
    ros::Subscriber objectPosition_subscriber;
    ros::Subscriber explorationCompletion_subscriber;
    tf::TransformListener listener;

    FirstRunNode()
    {
        n = ros::NodeHandle("~");
        begin = ros::Time::now();
        i = j = 0;
        exploration_start_stop = 0;
        exploration_completion = 0;
        object_detected_current = object_detected_prev = 0;
        objectDistanceGoal_old = 0.0;
        start_coordCalc = 0;
        start_pcl = 0;
        slowMotor_toUpdate = 0;
        current_state = INITIAL_STATE;
        previous_state = OBJ_UPDATION_STATE;
        current_state_exit = START_EXIT_PREPARATION;
        previous_state_exit = REACH_GOAL;
        objectDetection_publisher = n.advertise<std_msgs::String>("/espeak/string", 2);
        motorStop_publisher = n.advertise<std_msgs::Bool>("/pathFollow/slowDown", 1);
        cameraPcl_publisher = n.advertise<std_msgs::Bool>("/camera/start_pcl", 1);
        exploration_command_publisher = n.advertise<std_msgs::Bool>("/exploration/start_stop",1);
        objectCoordinateCalc_publisher = n.advertise<std_msgs::Bool>("/tf/start_calc", 1);
        robot_destination_publisher = n.advertise<geometry_msgs::PointStamped>("/robot/goal", 1);
        objectClass_subscriber = n.subscribe("/camera/object_class", 1, &FirstRunNode::objectClassCallback, this);
        objectDetection_subscriber = n.subscribe("/camera/object_detected", 1, &FirstRunNode::objectDetectionCallback, this);
        objectPosition_subscriber = n.subscribe("/map/objectCoord", 1, &FirstRunNode::objectPositionCallback, this);
        explorationCompletion_subscriber = n.subscribe("/brain/explorationCompletion", 1, &FirstRunNode::explorationCompletionCallback, this);
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

    void objectPositionCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        list[i].location = msg->point;
    }

    void explorationCompletionCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        exploration_completion = msg->data;
    }

    void pExplorationStateMachine()
    {

        if (ros::Time::now()- begin <= ros::Duration(60.0) && exploration_completion!=1 )
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
                    /*Start the PCL functionality of the camera */
                    start_pcl = 1;
                    msg_cameraPcl_startStop_bool.data = start_pcl;
                    cameraPcl_publisher.publish(msg_cameraPcl_startStop_bool);
                    /*Slow the motors to update the position of the object/replan the path*/
                    slowMotor_toUpdate = 1;
                    msg_motorSlowDown_bool.data = slowMotor_toUpdate;
                    motorStop_publisher.publish(msg_motorSlowDown_bool);
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
                if (INITIAL_STATE == previous_state)
                {
                    ROS_INFO("In state object updated");
                    current_state = OBJ_UPDATION_STATE;
                    previous_state = OBJ_DETECTION_STATE;
                    /****wait for object update in map****/
                    //service call
                    /*Update the object in the brain node's database*/
                    std::istringstream iss(classString);
                    do
                    {
                        iss >> list[i].colour_type[j];
                        std::cout<<"substring: "<<list[i].colour_type[j]<<std::endl;
                        j++;
                     } while (iss);
                    ROS_INFO("Position found: x=%.2f, y=%.2f", list[i].location.x,list[i].location.y);
                    j = 0;
                    i += 1;
                    /*Stop the PCL functionality of the camera */
                    start_pcl = 0;
                    msg_cameraPcl_startStop_bool.data = start_pcl;
                    cameraPcl_publisher.publish(msg_cameraPcl_startStop_bool);
                    /*Continue motors to run normally*/
                    slowMotor_toUpdate = 0;
                    msg_motorSlowDown_bool.data = slowMotor_toUpdate;
                    motorStop_publisher.publish(msg_motorSlowDown_bool);
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
            switch(current_state_exit)
            {
                case START_EXIT_PREPARATION:
                /* Find the object closer to the goal */
                for (int obj=0;obj<i;obj++)
                {
                    objectDistanceGoal_new = sqrt((list[obj].location.x)*(list[obj].location.x)+
                                              (list[obj].location.y)*(list[obj].location.y));
                    if (objectDistanceGoal_new>=objectDistanceGoal_old)
                    {
                        objectDistanceGoal_old = objectDistanceGoal_new;
                    }
                }
            }
            msg_robotDestination.header.frame_id = "map";
            msg_robotDestination.header.stamp = ros::Time::now();
            msg_robotDestination.point.x = 0.0;
            msg_robotDestination.point.x = 0.0;
            robot_destination_publisher.publish(msg_robotDestination);
        }
    }

    void pDestinationPublisher(geometry_msgs::PointStamped* destination)
    {
        msg_robotDestination.header = destination->header;
        msg_robotDestination.point = destination->point;
        robot_destination_publisher.publish(msg_robotDestination);
    }

private:
    std::string classString;
    bool object_detected_current, start_coordCalc, start_pcl, object_detected_prev;
    bool slowMotor_toUpdate, exploration_start_stop, pickup_object,exploration_completion;
    int current_state, previous_state;
    int current_state_exit, previous_state_exit;
    float objectDistanceGoal_new,objectDistanceGoal_old;
    std_msgs::String msg_eSpeak_string;
    geometry_msgs::PointStamped msg_robotDestination;
    std_msgs::Bool msg_tfObjCalc_bool,msg_exploration_startStop_bool,msg_cameraPcl_startStop_bool, msg_motorSlowDown_bool;
    std_msgs::Float32 msg_odomDiffDist_float;
    ros::Time begin;
    object_details list[10];
    int i,j;
};

bool setGoal(ras_project_brain::SetGoalPoint::Request  &req, ras_project_brain::SetGoalPoint::Response &res)
{
    FirstRunNode* first_run_obj = new FirstRunNode;
    geometry_msgs::PointStamped final_goal;
    final_goal.point.x = req.goalPoint.point.x;
    final_goal.point.y = req.goalPoint.point.y;
    final_goal.point.z = 0;
    final_goal.header.frame_id = req.goalPoint.header.frame_id;
    final_goal.header.stamp = req.goalPoint.header.stamp;
    //publish the goal value on the topic /robot/goal
    first_run_obj->pDestinationPublisher(&final_goal);
    res.error = 0;
    delete first_run_obj;
    return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "firstRun_node");

  FirstRunNode first_run_node;
  ros::ServiceServer service = first_run_node.n.advertiseService("set_robot_goal", setGoal);

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

