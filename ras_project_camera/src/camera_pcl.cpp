#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
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
  ros::Publisher pub_world_coord;
  ros::Publisher pub_pcl_filtered;
  int pixel_x, pixel_y, pcl_index;

public:
  camera_pcl()
  {
    sub = nh.subscribe ("/camera/depth_registered/points", 1, &camera_pcl::cloud_cb, this);
    sub_object_coord = nh.subscribe("/camera/image/object_coord", 100, &camera_pcl::object_coord_cb, this);
    pub_world_coord = nh.advertise<geometry_msgs::Point> ("camera/world_coord", 100);
    pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2> ("camera/pcl_filtered", 1);
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
  pcl::fromROSMsg(*cloud_msg, point_pcl);

  //sensor_msgs::PointCloud2 coord = *cloud_msg;
  
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

  /*
  // Convert msg to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);
  //std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  
  // Perform the VoxelGrid filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  vox.setInputCloud (cloudPtr);
  vox.setLeafSize (0.001, 0.001, 0.001);
  vox.filter (cloud_filtered_blob);

  //Create container for segmented data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (cloud_filtered_blob, *cloud_filtered);

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

  // Perform the Pass through filtering to limit z axis
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.5);

  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered_z);

  // Create the statistical outlier removal filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered_z);
  sor.setMeanK (100); //50 for 0.005 voxel grid
  sor.setStddevMulThresh (0.001); //0.001
  sor.filter (*cloud_filtered_);
  std::cerr << "PointCloud after Kmean: " << cloud_filtered_->width * cloud_filtered_->height << " data points." << std::endl;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered_, output);
  pub_pcl_filtered.publish (output);
*/
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