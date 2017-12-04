#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/Quaternion.h>
#include <ras_project_camera/QuaternionStamped.h>
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
#include <ctime>
#include <vector>

#include <math.h>
#include <pcl/common/transforms.h>

struct timespec start, finish;
double elapsed;

class detection
{

private:
public:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_barcode;
  ros::Subscriber sub_color_detected;
  ros::Publisher pub_trap_coord;
  ros::Publisher pub_battery_coord;
  ros::Publisher pub_pcl_filtered;

  pcl::PCLPointCloud2 cloud_blob;
  ros::Time lastReading;
  bool hasReading;
  bool hasBarcode;
  bool color_detected;
  int pixel_x, pixel_y;
  int min_size, min_battery_size;

  detection(): cloud_blob(), min_size(100), min_battery_size(200), hasBarcode(0), color_detected(0)
  {
    sub = nh.subscribe ("/camera/depth/points", 1, &detection::cloud_cb, this);
    sub_barcode = nh.subscribe ("/barcode", 1, &detection::barcode_cb, this);
    sub_color_detected = nh.subscribe ("/camera/object_detected", 1, &detection::color_detected_cb, this);
    pub_trap_coord = nh.advertise<geometry_msgs::PointStamped> ("/camera/trap_coord", 1);
    //pub_battery_coord = nh.advertise<geometry_msgs::Quaternion> ("/camera/battery_coord", 1);
    pub_battery_coord = nh.advertise<ras_project_camera::QuaternionStamped> ("/camera/battery_coord", 1);
    pub_pcl_filtered = nh.advertise<sensor_msgs::PointCloud2> ("/camera/pcl_filtered", 1);
    
    ros::NodeHandle nh("~");
    nh.getParam("min_size",min_size); 
    nh.getParam("min_battery_size",min_battery_size);
  }
  
  void barcode_cb (const std_msgs::String msg)
  {
    hasBarcode = 1;
  }

  void color_detected_cb (const std_msgs::Bool::ConstPtr& msg)
  {
    color_detected = msg->data;
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    // Convert msg to PCL data type
    hasReading = 1;
    lastReading = cloud_msg->header.stamp;
    pcl_conversions::toPCL(*cloud_msg, cloud_blob);
  }
};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "detection");
  detection ic;
  //Create container for original and filtered data
  pcl::PCLPointCloud2ConstPtr cloudPtr(&ic.cloud_blob);
  pcl::PCLPointCloud2 cloud_filtered_vox;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  std::vector<int> detection_count (3, 0);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!ic.hasReading)
    { 
        continue;
    }
    clock_gettime(CLOCK_MONOTONIC, &start);
    //ROS_INFO("Subscribed for Point Cloud");
    // Perform the VoxelGrid filtering
    vox.setInputCloud (cloudPtr);
    vox.setLeafSize (0.005, 0.005, 0.005); //.002
    vox.filter (cloud_filtered_vox);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (cloud_filtered_vox, *cloud_filtered_);

    // Perform the Pass through filtering to limit z axis
    pass.setInputCloud (cloud_filtered_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.5);
    pass.filter (*cloud_filtered_z);

    // Create the statistical outlier removal filtering object
    sor.setInputCloud (cloud_filtered_z);
    sor.setMeanK (50); //150 for 0.002 50 for 0.005 voxel grid
    sor.setStddevMulThresh (0.001); //0.001
    sor.filter (*cloud_filtered);

    // RANSAC
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    for (int k=0; k<1; ++k)
    {
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
    }
    // is the point cloud an object
    int object_size = cloud_filtered->width * cloud_filtered->height;
    ROS_INFO("Object size: %d", object_size);
    ROS_INFO("Min object size: %d", ic.min_size);

    geometry_msgs::PointStamped trap_coord;
    trap_coord.header.stamp = ic.lastReading;
    trap_coord.header.frame_id = "camera";

    //geometry_msgs::Quaternion battery_coord;
    ras_project_camera::QuaternionStamped battery_coord;
    battery_coord.header.stamp = ic.lastReading;
    battery_coord.header.frame_id = "camera";
    /*geometry_msgs::PointStamped battery_coord;
    battery_coord.header.stamp = ic.lastReading;
    battery_coord.header.frame_id = "camera";*/

    // trap detection 
    if (object_size > ic.min_size && ic.hasBarcode)
    {
      Eigen::Vector4f centroid;
      ROS_INFO("barcode cb: %d", ic.hasBarcode);
      pcl::compute3DCentroid(*cloud_filtered,centroid);
      ic.hasBarcode = 0;
      ROS_INFO("barcode cb: %d", ic.hasBarcode);
      trap_coord.point.x = centroid[2];
      trap_coord.point.y = centroid[0];
      trap_coord.point.z = centroid[1];
      ic.pub_trap_coord.publish (trap_coord);
    }

    // battery detection
    else if (object_size > ic.min_battery_size && !ic.color_detected && !ic.hasBarcode)
    {
      // find the orientation
      float z_min = cloud_filtered->points[0].z;
      int z_min_ind = 0;

      float x_max = cloud_filtered->points[0].x;
      int x_max_ind = 0;

      for (int m = 1; m<object_size; ++m)
      {
        if (cloud_filtered->points[m].z<z_min)
        {
          z_min_ind = m;
        }
        if (abs(cloud_filtered->points[m].x)>x_max)
        {
          x_max_ind = m;
        }
      }

      float z_value = cloud_filtered->points[x_max_ind].z - cloud_filtered->points[z_min_ind].z;
      float x_value = cloud_filtered->points[x_max_ind].x - cloud_filtered->points[z_min_ind].x;
      std::cout << "z_value " << z_value <<std::endl;
      std::cout << "x_value " << x_value <<std::endl;

      float angle = atan2(z_value, x_value)*180.0/M_PI;
      std::cout << "angle: " << angle <<std::endl;


      battery_coord.data.x = cloud_filtered->points[z_min_ind].z;
      battery_coord.data.y = cloud_filtered->points[z_min_ind].x;
      battery_coord.data.z = cloud_filtered->points[z_min_ind].y;
      battery_coord.data.w = angle;
      ic.pub_battery_coord.publish (battery_coord);
    }
    
    // publish filtered cloud for visualization
    sensor_msgs::PointCloud2 output;
    output.header.frame_id = "camera";
    pcl::toROSMsg(*cloud_filtered, output);
    ic.pub_pcl_filtered.publish (output);

    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    //std::cerr << "time of one loop "<< elapsed << std::endl;
    loop_rate.sleep();
    
  }
  return 0;
}
