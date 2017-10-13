#include<ros/ros.h>
#include <math.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "message_filters/subscriber.h"

class LaserToPointCloud{

    public:
        ros::NodeHandle n_;
        laser_geometry::LaserProjection projector;
        tf::TransformListener listener_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
        ros::Publisher scan_pub_;

        LaserToPointCloud(ros::NodeHandle n) : n_(n), laser_sub_(n_,"base_scan", 10), laser_notifier_(laser_sub_,listener_,"base_link",10)
    {
        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback,this,_1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;
        try
        {
            projector_.trasnformLaserScanToPointCloud("base_link",*scan_in,cloud,listener_);
        }
        catch (tf::TransformException& e)
        {
            std::cout << e.what();
            return;
        }
        scan_pub_.publish(cloud);
    }   
};

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_collision_detector");
    ros::NodeHandle n;
    LaserToPointCloud ltpc(n);

    ros::spin();

    return 0;
}
