#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main( int argc, char ** argv)
{
        ros::init(argc,argv,"uArmbase_broadcaster"); /*Check if it should be base or eef*/
        ros::NodeHandle n;
        tf::TransformBroadcaster br;

        ros::Rate rate(1000.0);
        while (n.ok())
        {
              /*x,y,z to be filled */
              br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.015, 0.0, 0.125)),
              ros::Time::now(),"robot", "uarm_base"));
              rate.sleep();
        }
        return 0;

}
