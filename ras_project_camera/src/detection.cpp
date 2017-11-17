#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <math.h>
class camera_pcl
{
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher object_flag_pub;
  //ros::Publisher pub_world_coord;
  ros::Publisher pub_pcl_filtered;
  int pixel_x, pixel_y, pcl_index;
  int min_object_size;

public:
  camera_pcl()
  {
    sub = nh.subscribe ("/camera/depth/points", 1, &camera_pcl::cloud_cb, this);
    object_flag_pub = nh.advertise<std_msgs::Bool>("/camera/object_detected",1);
    //pub_world_coord = nh.advertise<geometry_msgs::Point> ("camera/world_coord", 1);;
    pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2> ("camera/pcl_filtered", 1);
    ros::NodeHandle nh("~");
    nh.getParam("min_object_size",min_object_size);
  }

  void object_coord_cb (const geometry_msgs::Point::ConstPtr& object_coord_msg)
  {
    pixel_x = object_coord_msg->x;
    pixel_y = object_coord_msg->y;
    //std::cerr << "x y" <<pixel_x << pixel_y << std::endl;
  }
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
  
  //Create container for original and filtered data
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered_vox;
  // Convert msg to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);
  //std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  ROS_INFO("Subscribed for Point Cloud");
  // Perform the VoxelGrid filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud (cloudPtr);
  vox.setLeafSize (0.005, 0.005, 0.005); //.002
  vox.filter (cloud_filtered_vox);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (cloud_filtered_vox, *cloud_filtered_);

  // Perform the Pass through filtering to limit z axis
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered_);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered_z);


  // Create the statistical outlier removal filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered_z);
  sor.setMeanK (50); //150 for 0.002 50 for 0.005 voxel grid
  sor.setStddevMulThresh (0.005); //0.001
  sor.filter (*cloud_filtered);

  //Create container for segmented data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // RANSAC
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
  // is the point cloud an object
  int object_size = cloud_filtered->width * cloud_filtered->height;
  ROS_INFO("Object size: %d", object_size);
  ROS_INFO("Min object size: %d", min_object_size);

  std_msgs::Bool object_flag;
  if (object_size > min_object_size ) //&& object_size < min_object_size
 {
  ROS_INFO("Object detected!");
  object_flag.data = 1;
  object_flag_pub.publish(object_flag);

  /*float x_points = 0.0;
  float y_points = 0.0;
  float z_points = 0.0;
  //Find mean distance to the object
  for (int j = 0; j < object_size; ++j)
  {
     x_points += cloud_filtered->points[j].x;
     y_points += cloud_filtered->points[j].y;
     z_points += cloud_filtered->points[j].z;
  }
  float x_point = x_points/float(object_size);
  float y_point = y_points/float(object_size);
  float z_point = z_points/float(object_size);

  //transfet=r to world coord
  float x = z_point;
  float y = x_point;
  float z = y_point;
  std::cerr << "x y z " << x <<" "<< y <<" "<< z << std::endl;
  // Publish
  geometry_msgs::Point coord_from_camera;

  coord_from_camera.x = x;
  coord_from_camera.y = y;
  coord_from_camera.z = z;
  std::cerr << " object coords " << x << " " << y << " " << z << std::endl;
  pub_world_coord.publish (coord_from_camera);*/
  }
  else 
  {
    object_flag.data = 0;
    object_flag_pub.publish(object_flag);
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
  pub_pcl_filtered.publish (output);

  }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "camera_pcl");
  camera_pcl ic;

  // Spin
  ros::spin ();
  return 0;
}