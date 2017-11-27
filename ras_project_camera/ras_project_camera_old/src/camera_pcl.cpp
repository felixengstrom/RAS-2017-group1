#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <math.h>
class camera_pcl
{
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_object_coord;
  ros::Subscriber sub_object_detected;
  ros::Publisher pub_world_coord;
  ros::Publisher pub_pcl_filtered;
  int pixel_x, pixel_y, pcl_index;
  //bool object_detected;

public:
  camera_pcl()
  {
    sub = nh.subscribe ("/camera/depth_registered/points", 1, &camera_pcl::cloud_cb, this);
    sub_object_coord = nh.subscribe("/camera/object_coord", 100, &camera_pcl::object_coord_cb, this);
    //sub_object_detected = nh.subscribe("/object_detected", 1, &camera_pcl::detectionCb, this);
    pub_world_coord = nh.advertise<geometry_msgs::Point> ("camera/world_coord", 100);
    pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2> ("camera/pcl_filtered", 1);
  }

  void detectionCb(const std_msgs::Bool::ConstPtr& msg)
  {
    bool object_detected = msg->data;
    std::cerr << "object_detected" << object_detected << std::endl;
  }

  void object_coord_cb (const geometry_msgs::Point::ConstPtr& object_coord_msg)
  {
    pixel_x = object_coord_msg->x;
    pixel_y = object_coord_msg->y;
    std::cerr << "x y" <<pixel_x << pixel_y << std::endl;
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
  
  //Create container for original and filtered data
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered_blob;

  //Real world coord
  pcl::PointCloud<pcl::PointXYZ> point_pcl;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    //if (object_detected == 1)
       // std::cerr << "object_detected" << object_detected << std::endl;
      //{
  pcl::fromROSMsg(*cloud_msg, point_pcl);
  
  int width = point_pcl.width;
  int height = point_pcl.height;

  pcl_index = pixel_y*width + pixel_x;
  //ROS_INFO("pcl_index: %i", pcl_index);
  pcl::PointXYZ p = point_pcl.at(pcl_index);
  ROS_INFO("after at index");
  //in case p consist NaN
  int dx[] = {0,1,0,-1};
  int dy[] = {1,0,-1,0};
  if (pixel_x >= width) pixel_x = width-1;
  if (pixel_x == width -1) dx[1] = 0;
  if (pixel_x == 0) dx[3] = 0;
  if (pixel_y >= height) pixel_y = height-1;
  if (pixel_y == height -1) dy[1] = 0;
  if (pixel_y == 0) dy[3] = 0;

  for (int i = 0; isnan(p.x) && i < 4; ++i)
{
  pcl_index = (pixel_y+dy[i])*width + (pixel_x+dx[i]);
  p = point_pcl.at(pcl_index);
  std::cerr << "value was non " << std::endl;
} 
  
  float x, y, d, z;
  x = p.z;
  y = p.x;
  z = p.y;
  std::cerr << "x y z " << x <<" "<< y <<" "<< z << std::endl;

  geometry_msgs::Point coord_from_camera;

  coord_from_camera.x = x;
  coord_from_camera.y = y;
  coord_from_camera.z = z;

  pub_world_coord.publish (coord_from_camera);
  loop_rate.sleep();
  ros::spinOnce();
  //}
}
}
};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_pcl");
  camera_pcl ic;
  ros::spin ();
  return 0;
}