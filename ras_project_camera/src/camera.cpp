#include <ros/ros.h>
// PCL specific includes
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <iostream>

class camera
{
  
public:
  
  ros::NodeHandle nh;
  ros::Subscriber sub_detection;
  ros::Subscriber sub_color_detection;
  ros::Publisher pub_detected;
  ros::Publisher pub_world_coord;
  ros::Publisher pub_pcl_filtered;
  bool detected, object_detected;

  detection(): 
  {
    sub_detection = nh.subscribe ("/camera/detected", 1, &detection::detected_cb, this);
    sub_color_detection = nh.subscribe ("/camera/object_detected", 1, &detection::object_detected_cb, this);
    pub_classify = nh.advertise<std_msgs::Bool>("/camera/classify",1);
    pub_world_coord = nh.advertise<geometry_msgs::Point> ("/camera/detection_coord", 1);;
    pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2> ("camera/pcl_filtered", 1);
    
    ros::NodeHandle nh("~");
    nh.getParam("min_object_size",min_object_size); 
  }
  
  void detected_cb (const std_msgs::Bool::ConstPtr& msg)
  {
    // Convert msg to PCL data type
    detected = msg->data;
  }

  void object_detected_cb (const std_msgs::Bool::ConstPtr& msg)
  {
    // Convert msg to PCL data type
    object_detected = msg->data;
  }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera");
  camera ic;
  
  ros::Rate loop_rate(10);
      while (ros::ok())
      {
        ros::spinOnce();
        if (detected || object_detected)
        {
          std_msgs::Bool classify;
          classify = 1;
          ic.pub_classify.publish(classify);
        }
        
        loop_rate.sleep();
        
      }
  return 0;
}
