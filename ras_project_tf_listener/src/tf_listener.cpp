#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <ras_project_uarm/MoveArmCartesian.h>

#define TRUE 1

static bool objectPickUp;

void chatterCallback(const std_msgs::Bool::ConstPtr& msg)
{
  objectPickUp = msg->data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<ras_project_uarm::MoveArmCartesian>("/uarm/moveToPose");
  ros::Publisher objectMap_publisher = node.advertise<geometry_msgs::PointStamped>("/map/objectCoord", 1);
  ros::Subscriber sub = node.subscribe("/tf/pickup_obj", 1, chatterCallback);
  ras_project_uarm::MoveArmCartesian srv;
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform_arm, transform_map;
    if(objectPickUp == TRUE)
    {
        try
        {
            listener.waitForTransform("uarm_base","object",ros::Time(0), ros::Duration(2));
            listener.lookupTransform("uarm_base", "object", ros::Time(0), transform_arm);
            //listener.waitForTransform("map","object",ros::Time(0), ros::Duration(2));
            //listener.lookupTransform("map", "object", ros::Time(0), transform_map);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

     geometry_msgs::PointStamped uArmObjPos_msg;
     geometry_msgs::PointStamped mapObjPos_msg;
     uArmObjPos_msg.point.x = transform_arm.getOrigin().x();
     uArmObjPos_msg.point.y = transform_arm.getOrigin().y();
     uArmObjPos_msg.point.z = transform_arm.getOrigin().z();
     uArmObjPos_msg.header.frame_id = "uArm_base";
     uArmObjPos_msg.header.stamp = ros::Time::now();

     /*mapObjPos_msg.point.x = transform_map.getOrigin().x();
     mapObjPos_msg.point.y = transform_map.getOrigin().y();
     mapObjPos_msg.point.z = transform_map.getOrigin().z();
     mapObjPos_msg.header.frame_id = "map";
     mapObjPos_msg.header.stamp = ros::Time::now();

     ROS_INFO("object -----> uArm_base: (%.2f, %.2f, %.2f) at time %.2f",
     uArmObjPos_msg.point.x, uArmObjPos_msg.point.y, uArmObjPos_msg.point.z, uArmObjPos_msg.header.stamp.toSec());
     srv.request.point = uArmObjPos_msg;*/
     objectPickUp = (!TRUE);
     ROS_INFO("Object PickUp value:%d", objectPickUp);
     if(client.call(srv))
     {
        ROS_INFO("Error value: %d",srv.response.error);
     }
     //objectMap_publisher.publish(mapObjPos_msg);
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
