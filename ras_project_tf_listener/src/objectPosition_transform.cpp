/* This file transforms the object position wrt the robot frame */

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#define TRUE 1

static geometry_msgs::PointStamped object_point;
static bool object_present;
static geometry_msgs::PointStamped trap_point;
static bool trap_present;

void objectPosition_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  /* Here the x,y,z position of the object wrt the camera frame are taken */
  object_point.point.x = msg->point.x;
  object_point.point.y = -msg->point.y;
  object_point.point.z = msg->point.z;
  object_point.header.frame_id = msg->header.frame_id;
  object_point.header.stamp = msg->header.stamp;
}
void objectDetection_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    object_present = msg->data;
    if (trap_present)
	trap_present = false;
    /*if (battery_present)
	battery_present = false;*/
}

void trapPosition_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (!trap_present)
	trap_present = true;
    /*if (battery_present)
	battery_present = false;*/
    trap_point.point.x = msg->point.x;
    trap_point.point.y = -msg->point.y;
    trap_point.point.z = msg->point.z;
    trap_point.header.frame_id = msg->header.frame_id;
    trap_point.header.stamp = msg->header.stamp;
}

/*void batteryPosition_Callback(const greometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    if (trap_present)
	trap_present = false;
    if (!battery_present)
	battery_present = true;
    battery_point.point.x = msg->point.x;
    battery_point.point.y = -msg->point.y;
    battery_point.point.z = msg->point.z;
    battery_point.header.frame_id = msg->header.frame_id;
    battery_point.header.stamp = msg->header.stamp;
}*/

void transformPoint(const tf::TransformListener& listener, ros::Publisher map_publisher)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    tf::StampedTransform transform_map;
    //geometry_msgs::PointStamped mapObjPos_msg;

    //we'll create a point in the camera frame that we'd like to transform to the robot frame
/*    object_point.header.frame_id = "camera";
 
    //we'll just use the most recent transform available for our simple example
    object_point.header.stamp = ros::Time(0);*/
    if (trap_present && trap_point.header.stamp.toSec() + 1 < ros::Time::now().toSec())
	trap_present = false;
    if(TRUE == object_present || trap_present)
    {
        try
        {
            geometry_msgs::PointStamped point_base;
	    if (object_present == TRUE)
	    {
            	listener.transformPoint("robot", object_point, point_base);
		point_base.header.stamp = object_point.header.stamp;
  	    }
	    else
	    {
		listener.transformPoint("robot", trap_point, point_base);
		point_base.header.stamp = trap_point.header.stamp;
	    }
            ROS_INFO("object: (%.2f, %.2f. %.2f) -----> robot: (%.2f, %.2f, %.2f) at time %.2f",
            object_point.point.x, object_point.point.y, object_point.point.z,
            point_base.point.x, point_base.point.y, point_base.point.z, point_base.header.stamp.toSec());
            transform.setOrigin(tf::Vector3(point_base.point.x,point_base.point.y,point_base.point.z));
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

       /* mapObjPos_msg.point.x = transform_map.getOrigin().x();
        mapObjPos_msg.point.y = transform_map.getOrigin().y();
        mapObjPos_msg.point.z = 0.0;
        mapObjPos_msg.header.frame_id = "map";
        mapObjPos_msg.header.stamp = ros::Time::now();*/
	geometry_msgs::TransformStamped message;
	tf::transformStampedTFToMsg(&transform_map, &message);
	message.header.stamp = point_base.header.stamp;

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
    //ros::Publisher batteryMap_publisher = n.advertise<geometry_msgs::TransformStamped>("/map/batteryCoord", 1);
    object_present = false;
    trap_present = false;

   //we'll transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener), boost::ref(objectMap_publisher)));
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener), boost::ref(trapMap_publisher)));
   //ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformQuaternion, boost::ref(listener), boost::ref(batteryMap_publisher)));
 
   ros::spin();
 
}
