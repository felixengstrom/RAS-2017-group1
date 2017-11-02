/* This file transforms the object position wrt the robot frame */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
  
void transformPoint(const tf::TransformListener& listener)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    //we'll create a point in the camera frame that we'd like to transform to the robot frame
    geometry_msgs::PointStamped object_point;
    object_point.header.frame_id = "camera";
 
    //we'll just use the most recent transform available for our simple example
    object_point.header.stamp = ros::Time(0);

    /* Here the x,y,z position of the object wrt the camera frame are taken */
    object_point.point.x = 0.1;
    object_point.point.y = 0.0;
    object_point.point.z = -0.17;
 
try{
    geometry_msgs::PointStamped object_point_base;
    listener.transformPoint("robot", object_point, object_point_base);
  
    ROS_INFO("object: (%.2f, %.2f. %.2f) -----> robot: (%.2f, %.2f, %.2f) at time %.2f",
    object_point.point.x, object_point.point.y, object_point.point.z,
    object_point_base.point.x, object_point_base.point.y, object_point_base.point.z, object_point_base.header.stamp.toSec());
    transform.setOrigin(tf::Vector3(object_point_base.point.x,object_point_base.point.y,object_point_base.point.z));
    q.setRPY(0,0.0,0);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"robot","object"));
}
catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"camera\" to \"robot\": %s", ex.what());
    }
    
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_transform");
    ros::NodeHandle n;
    tf::TransformListener listener(ros::Duration(100));
  
   //we'll transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
 
   ros::spin();
 
}
