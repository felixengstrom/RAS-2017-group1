#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <ras_project_uarm/MoveArmCartesian.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<ras_project_uarm::MoveArmCartesian>("/uarm/moveToPose");
  ras_project_uarm::MoveArmCartesian srv;
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("uarm_base", "object",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PointStamped uArmObjPos_msg;
    uArmObjPos_msg.point.x = transform.getOrigin().x();
    uArmObjPos_msg.point.y = transform.getOrigin().y();
    uArmObjPos_msg.point.z = transform.getOrigin().z();
    uArmObjPos_msg.header.frame_id = "uArm_base";
    uArmObjPos_msg.header.stamp = ros::Time();
    ROS_INFO("object -----> uArm_base: (%.2f, %.2f, %.2f) at time %.2f",
    uArmObjPos_msg.point.x, uArmObjPos_msg.point.y, uArmObjPos_msg.point.z, uArmObjPos_msg.header.stamp.toSec());
    srv.request.point = uArmObjPos_msg;
    if(client.call(srv))
        ROS_INFO("%d",srv.response.error);

    rate.sleep();
  }
  return 0;
}
