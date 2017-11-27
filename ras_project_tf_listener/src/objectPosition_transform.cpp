/* This file transforms the object position wrt the robot frame */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#define TRUE 1

static geometry_msgs::PointStamped object_point;
static bool object_present;

void objectPosition_Callback(const geometry_msgs::Point::ConstPtr& msg)
{
  /* Here the x,y,z position of the object wrt the camera frame are taken */
  object_point.point.x = msg->x;
  object_point.point.y = -msg->y;
  object_point.point.z = msg->z;
}
void objectDetection_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    object_present = msg->data;
}

void transformPoint(const tf::TransformListener& listener, ros::Publisher objectMap_publisher)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    tf::StampedTransform transform_map;
    geometry_msgs::PointStamped mapObjPos_msg;

    //we'll create a point in the camera frame that we'd like to transform to the robot frame
    object_point.header.frame_id = "camera";
 
    //we'll just use the most recent transform available for our simple example
    object_point.header.stamp = ros::Time(0);

    if(TRUE == object_present)
    {
        try
        {
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
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"camera\" to \"robot\": %s", ex.what());
        }
        try
        {

            listener.waitForTransform("map","object",ros::Time(0), ros::Duration(2));
            listener.lookupTransform("map", "object", ros::Time(0), transform_map);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        mapObjPos_msg.point.x = transform_map.getOrigin().x();
        mapObjPos_msg.point.y = transform_map.getOrigin().y();
        mapObjPos_msg.point.z = 0.0;
        mapObjPos_msg.header.frame_id = "map";
        mapObjPos_msg.header.stamp = ros::Time::now();

        objectMap_publisher.publish(mapObjPos_msg);
    }
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_transform");
    ros::NodeHandle n;
    tf::TransformListener listener(ros::Duration(10));
    ros::Subscriber sub_position = n.subscribe("camera/world_coord", 1, objectPosition_Callback);
    ros::Subscriber sub_detection = n.subscribe("/tf/start_calc", 1, objectDetection_Callback);
    ros::Publisher objectMap_publisher = n.advertise<geometry_msgs::PointStamped>("/map/objectCoord", 1);

   //we'll transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener), boost::ref(objectMap_publisher)));
 
   ros::spin();
 
}
