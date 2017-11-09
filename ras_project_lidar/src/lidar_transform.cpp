#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>


//class lidar_transform
//{
//
//    private:
//         ros::NodeHandle n;
//         ros::Publisher lidar_pub;
//         ros::Subscriber sub;
//         tf::Transform transform;
//         tf::TransformBroadcaster br;
//         tf::Quaternion q;
//         float x,y,z,zrot;
//
//    public:
//         lidar_transform()
//         {
//            x=8.0;
//            y=0.0;
//            z=0.0;
//            zrot=acos(-1);
//            n = ros::NodeHandle();
//            sub = n.subscribe("/scan", 10 , &lidar_transform::LidarCallback,this);
//
//         
//         }
//        
//         void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
//
//
//}
//
//void lidar_transform::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
//{
//
//    transform.setOrigin(tf::Vector3(x,y,0.0));
//    transform.setRPY(0.0,0.0,zrot);
//    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"robot","lidar"));
//}
//
int main( int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_broadcaster");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    
    float zrot=acos(-1)*0.97;
    ros::Rate rate(10.0);
    while (n.ok())
    {
        transform.setOrigin(tf::Vector3(0.0,-0.07,0.0));
        q.setRPY(0,0,zrot);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"robot","laser"));
        rate.sleep();
    }

   return 0; 

}
