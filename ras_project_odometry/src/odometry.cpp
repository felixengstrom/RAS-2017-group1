#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "phidgets/motor_encoder.h"

const static float PI = acos(-1);
class OdometryNode
{
    private:
        ros::NodeHandle n_;
        ros::Publisher odom_pub; 
        tf::Transform transform;
        tf::TransformBroadcaster br;
        tf::Quaternion q;

    public:
        OdometryNode() 
        {
            delta_t = 0.008;
            n_ = ros::NodeHandle();
        }
        void PublishVelocity();
        {

            float control_freq = 1/delta_t;
            float est_wl = last_change_left*2*PI*control_freq/ticks_per_rev;
            float est_wr = last_change_right*2*PI*control_freq/ticks_per_rev;

            float v = wheel_radius*(est_wl + est_wr)/2;
            float omega = wheel_radius*(est_wr-est_wl)/2;

            transform.setOrigin( tf::Vector3(delta_t*v, 0.0, 0.0) );
            q.setRPY(0, 0, omega*delta_t);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
        }
};
void OdometryNode::PublishVelocity();
{

    transform.setOrigin( tf::Vector3(delta_t*v, 0.0, 0.0) );
    q.setRPY(0, 0, omega*delta_t);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
}
int main(int argc, char ** argv){
    ros::init(argc, argv, "odometry");
    OdometryNode on  = OdometryNode();
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        on.PublishVelocity();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//#include <messege_filters/subscriber.h>
//#include <messege_filters/syncronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
        //message_filters::Subscriber<phidgets::motor_encoder> left_encoder;
       //message_filters::Subscriber<phidgets::motor_encoder> right_encoder;
       // typedef sync_policies::ApproximateTime<phidgets::motor_encoder, phidgets::motor_encoder> MySyncPolicy;
       // // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
       // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_encoder, right_encoder);

       // sync.registerCallback(boost::bind(&EncoderCallback, _1, _2));
