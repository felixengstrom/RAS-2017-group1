/* This file transforms the object position wrt the robot frame */

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#define TRUE 1

static geometry_msgs::PoseStamped object_point;
static bool object_present;
static geometry_msgs::PoseStamped trap_point;
static bool trap_present;
static geometry_msgs::PoseStamped battery_point;
static bool battery_present;
static bool detection_activated;

void objectPosition_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  /* Here the x,y,z position of the object wrt the camera frame are taken */
  object_point.pose.position.x = msg->point.x;
  object_point.pose.position.y = -msg->point.y;
  object_point.pose.position.z = msg->point.z;
  object_point.pose.orientation.w = 1.0;
  object_point.header.frame_id = "camera";//msg->header.frame_id;
  object_point.header.stamp = msg->header.stamp;
  object_present = true;
  if (trap_present)
	trap_present = false;
  if (battery_present)
	battery_present = false;
}
void objectDetection_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    detection_activated = msg->data;
}

void trapPosition_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    trap_present = true;
    if (battery_present)
	battery_present = false;
    trap_point.pose.position.x = msg->point.x;
    trap_point.pose.position.y = -msg->point.y;
    trap_point.pose.position.z = msg->point.z;
    trap_point.pose.orientation.w = 1.0;
    trap_point.header.frame_id = "camera";//msg->header.frame_id;
    trap_point.header.stamp = msg->header.stamp;
    if (object_present)
	object_present = false;
}

void batteryPosition_Callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    if (trap_present)
	trap_present = false;
    if (object_present)
	object_present = false;
    battery_present = true;
    battery_point.pose.position.x = msg->quaternion.x;
    battery_point.pose.position.y = -msg->quaternion.y;
    battery_point.pose.position.z = msg->quaternion.z;
    battery_point.pose.orientation.w = 1.0;
    battery_point.header.frame_id = "camera";//msg->header.frame_id;
    battery_point.header.stamp = msg->header.stamp;
}

void transformPoint(const tf::TransformListener& listener, ros::Publisher object_pub, ros::Publisher trap_pub, ros::Publisher battery_pub)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    tf::StampedTransform transform_map;
    ros::Publisher map_publisher;
    //geometry_msgs::PointStamped mapObjPos_msg;

    //we'll create a point in the camera frame that we'd like to transform to the robot frame
/*    object_point.header.frame_id = "camera";
 
    //we'll just use the most recent transform available for our simple example
    object_point.header.stamp = ros::Time(0);*/
    /*if (trap_present && trap_point.header.stamp.toSec() + 2 < ros::Time::now().toSec())
	trap_present = false;
    if (object_present && object_point.header.stamp.toSec() + 2 < ros::Time::now().toSec())
	object_present = false;
    if (battery_present && battery_point.header.stamp.toSec() + 2 < ros::Time::now().toSec())
	battery_present = false;*/
    if(detection_activated && (object_present || trap_present || battery_present))
    {
	geometry_msgs::PoseStamped pose_base;
        try
        {
	    if (object_present)
	    {
            	listener.transformPose("map", object_point, pose_base);
		pose_base.header.stamp = object_point.header.stamp;
		map_publisher = object_pub;
		ROS_INFO("Object !");
		object_present = false;
  	    }
	    else if (trap_present)
	    {
		listener.transformPose("map", trap_point, pose_base);
		pose_base.header.stamp = trap_point.header.stamp;
		map_publisher = trap_pub;
		trap_present = false;
	    }
	    else
	    {
		listener.transformPose("map", battery_point, pose_base);
		pose_base.header.stamp = battery_point.header.stamp;
		map_publisher = battery_pub;
		battery_present = false;
	    }
            //ROS_INFO("object: (%.2f, %.2f. %.2f) -----> robot: (%.2f, %.2f, %.2f) at time %.2f",
            //object_point.pose.position.x, object_point.pose.position.y, object_point.pose.position.z,
            //pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z, pose_base.header.stamp.toSec());
            //transform.setOrigin(tf::Vector3(point_base.point.x,point_base.point.y,point_base.point.z));
            //q.setRPY(0,0.0,0);
            //transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"robot","object"));
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"camera\" to \"robot\": %s", ex.what());
        }
        try
        {
            //listener.waitForTransform("map","object",ros::Time(0), ros::Duration(2));
            //listener.lookupTransform("map", "object", ros::Time(0), transform_map);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        /*mapObjPos_msg.point.x = transform_map.getOrigin().x();
        mapObjPos_msg.point.y = transform_map.getOrigin().y();
        mapObjPos_msg.point.z = 0.0;
        mapObjPos_msg.header.frame_id = "map";
        mapObjPos_msg.header.stamp = ros::Time::now();*/
	geometry_msgs::TransformStamped message;
	//tf::transformStampedTFToMsg(transform_map, message);
	message.header.stamp = pose_base.header.stamp;
	message.child_frame_id = "map";
	message.transform.translation.x = pose_base.pose.position.x;
	message.transform.translation.y = pose_base.pose.position.y;
	message.transform.rotation.z = pose_base.pose.orientation.z;
	message.header.frame_id = "camera";
        map_publisher.publish(message);
    }
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_position_transform");
    ros::NodeHandle n;
    tf::TransformListener listener(ros::Duration(10));
    ros::Subscriber sub_object_position = n.subscribe("/camera/world_coord", 1, objectPosition_Callback);
    ros::Subscriber sub_detection = n.subscribe("/tf/start_calc", 1, objectDetection_Callback);
    ros::Subscriber sub_trap_position = n.subscribe("/camera/trap_coord", 1, trapPosition_Callback);
    ros::Publisher objectMap_publisher = n.advertise<geometry_msgs::TransformStamped>("/map/objectCoord", 1);
    ros::Publisher trapMap_publisher = n.advertise<geometry_msgs::TransformStamped>("/map/trapCoord", 1);
    ros::Publisher batteryMap_publisher = n.advertise<geometry_msgs::TransformStamped>("/map/batteryCoord", 1);
    object_present = false;
    trap_present = false;
    battery_present = false;
    detection_activated = true;

   //we'll transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener), boost::ref(objectMap_publisher), boost::ref(trapMap_publisher), boost::ref(batteryMap_publisher)));
   //ros::Timer timer2 = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener), boost::ref(trapMap_publisher)));
   //ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformQuaternion, boost::ref(listener), boost::ref(batteryMap_publisher)));
 
   ros::spin();
 
}
