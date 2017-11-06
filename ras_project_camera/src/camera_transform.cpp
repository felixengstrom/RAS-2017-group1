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

        ros::Rate rate(1000.0);
        while (n.ok())
        {
            transform.setOrigin(tf::Vector3(0.13,0.0,0.125));
            //q.setRPY(0,-0.105,0);
            //transform.setOrigin(tf::Vector3(.274,0.0,-0.049));
            q.setRPY(0,-3.142,0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"robot","camera"));
            rate.sleep();
        }

    return 0;

}
