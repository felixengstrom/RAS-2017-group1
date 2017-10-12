#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;
  ros::Publisher uArmObj_position =
      node.advertise<geometry_msgs::Point>("uArm/moveToPose", 10);
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("uArm_base", "camera",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Point uArmObjPos_msg;
    uArmObjPos_msg.Point.x = transform.getOrigin().x();
    uArmObjPos_msg.Point.y = transform.getOrigin().y();
    uArmObjPos_msg.Point.z = transform.getOrigin().z();
    uArmObj_position.publish(uArmObjPos_msg);

    rate.sleep();
  }
  return 0;
};
