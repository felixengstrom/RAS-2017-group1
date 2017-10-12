#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main( int argc, char ** argv)
{
    ros::init(argc,argv,"camera_broadcaster");
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    ros::Rate rate(10.0);
    while (n.ok())
    {
        transform.setOrigin(tf::Vector3(0.0,0.0,0.0));/*Set the values after measuring*/
        q.setRPY(0,0,0);/*Set the angle of orientation*/
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"robot","camera"));
        rate.sleep();
    }

   return 0;

}
