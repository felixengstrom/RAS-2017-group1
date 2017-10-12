#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main( int argc, char ** argv)
{
    ros::init(argc,argv,"uArmbase_broadcaster"); /*Check if it should be base or eef*/
    ros::NodeHandle n;
    tf::TransformBroadcaster br;

    ros::Rate rate(10.0);
    while (n.ok())
    {
        /*x,y,z to be filled */
        br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                         ros::Time::now(),"robot", "uArm_base"));/*eef or base?*/
        rate.sleep();
    }

   return 0;

}
