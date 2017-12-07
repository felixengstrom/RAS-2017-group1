#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
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
public:
bool has_coord_msg;
bool has_cloud;
pcl::PointCloud<pcl::PointXYZ> point_pcl;
int pixel_x, pixel_y;
ros::NodeHandle nh;
ros::Subscriber sub;
ros::Subscriber sub_object_coord;
ros::Subscriber sub_object_detected;
ros::Publisher pub_world_coord;
bool object_detected;
ros::Time lastReading;

camera_pcl(): point_pcl(), object_detected(), pixel_x(100), pixel_y(200), has_coord_msg(0), has_cloud(0)
{
sub = nh.subscribe ("/camera/depth_registered/points", 1, &camera_pcl::cloud_cb, this);
sub_object_coord = nh.subscribe("/camera/object_coord", 10, &camera_pcl::object_coord_cb, this);
//sub_object_detected = nh.subscribe("/camera/object_detected", 1, &camera_pcl::detection_cb, this);
pub_world_coord = nh.advertise<geometry_msgs::PointStamped> ("/camera/world_coord", 1);
}

/*void detection_cb(const std_msgs::Bool::ConstPtr& msg)
{
object_detected = msg->data;
//std::cerr << "object_detected" << object_detected << std::endl;
}*/

void object_coord_cb (const geometry_msgs::PointStamped::ConstPtr& object_coord_msg)
{
pixel_x = object_coord_msg->point.x;
pixel_y = object_coord_msg->point.y;
//std::cerr << "x y" <<pixel_x << pixel_y << std::endl;
lastReading = object_coord_msg->header.stamp;
has_coord_msg = true;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
pcl::fromROSMsg(*cloud_msg, point_pcl);
has_cloud = true;
std::cerr << "has cloud " << has_cloud << std::endl;  
}

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "camera_pcl");
  camera_pcl ic;
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    
    //std::cerr << ic.object_detected << " " << ic.has_cloud << " " << ic.has_coord_msg << std::endl;
    if (ic.has_cloud && ic.has_coord_msg)//(ic.object_detected && ic.has_cloud && ic.has_coord_msg)
      {
        int width = ic.point_pcl.width;
        int height = ic.point_pcl.height;

        int pcl_index = ic.pixel_y*width + ic.pixel_x;
        pcl::PointXYZ p = ic.point_pcl.at(pcl_index);

        //in case p consist NaN
        int dx[] = {0,1,0,-1};
        int dy[] = {1,0,-1,0};
        if (ic.pixel_x >= width) ic.pixel_x = width-1;
        if (ic.pixel_x == width -1) dx[1] = 0;
        if (ic.pixel_x == 0) dx[3] = 0;
        if (ic.pixel_y >= height) ic.pixel_y = height-1;
        if (ic.pixel_y == height -1) dy[1] = 0;
        if (ic.pixel_y == 0) dy[3] = 0;

        for (int i = 0; isnan(p.x) && i < 4; ++i)
        {
          pcl_index = (ic.pixel_y+dy[i])*width + (ic.pixel_x+dx[i]);
          p = ic.point_pcl.at(pcl_index);
          //std::cerr << "x y z  " <<p.x << " "<< p.y <<" " <<p.z<< std::endl;
        } 
        
        geometry_msgs::PointStamped coord_from_camera;
	
        coord_from_camera.header.stamp = ic.lastReading;
        coord_from_camera.point.x = p.z;
        coord_from_camera.point.y = p.x;
        coord_from_camera.point.z = p.y;
        ic.pub_world_coord.publish (coord_from_camera);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
