#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include "uarm/MoveToJoints.h"

static const double PI = acos(-1);

struct AngleSetting{
    double j0;
    double j1;
    double j2;
    double j3;
};
double radiansToDegrees(double radians)
{
    return (radians/(2*acos(-1)))*360;
}
double sgn(double x)
{
    return (x>0)-(x<0);
}
class UarmController{
    private:
        double j0_start, j1_start, j2_start, j3_start;
        double d0, d1, d2, d3, d4, d5;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::ServiceClient client;
    
    public:
        void moveArm(double x, double y, double z);
        UarmController(ros::NodeHandle& nh);
        void moveToPointCallback(const geometry_msgs::Point::ConstPtr& msg);
        AngleSetting invKinematic(double x,double y,double z);
};

UarmController::UarmController(ros::NodeHandle& nh)
{
    n = nh;
    ros::NodeHandle n_("~");

    j0_start = 80;
    j1_start = 80;
    j2_start = -27;
    j3_start  = 80;

    n_.getParam("j0_start", j0_start);
    n_.getParam("j1_start", j1_start);
    n_.getParam("j2_start", j2_start);
    n_.getParam("j3_start", j3_start);

    
    d0 = 9;
    d1 = 2.5;
    d2 = 14.85;
    d3 = 16;
    d4 = 3.5;
    d5 = 6;

    n_.getParam("d0", d0);
    n_.getParam("d1", d1);
    n_.getParam("d2", d2);
    n_.getParam("d3", d3);
    n_.getParam("d4", d4);
    n_.getParam("d5", d5);

    client = n.serviceClient<uarm::MoveToJoints>("/uarm/move_to_joints");

    sub = n.subscribe("uarm/moveToPoint",1, &UarmController::moveToPointCallback, this);
}

void UarmController::moveToPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{

    this->moveArm(msg->x,msg->y,msg->z);
    ROS_INFO("point to move to x:%f, y:%f, z:%f", msg->x, msg->y, msg->z);
}

AngleSetting UarmController::invKinematic(double x,double y,double z)
{   
    //Calculate first angle
    //for these calculations the j1 arm will be recalculated to be in a 
    //vertical position when j1 angle is 0, j2 will be recalculated to
    //be horizontal when j2 is zero
    AngleSetting angs;

    //Calculate base rotation angle
    double non_relative_j0 = atan(y/x);
    angs.j0 =  -radiansToDegrees(non_relative_j0) + this->j0_start;

    //Calculate distance between j1 and joint on end effectoror
    double h_hight = z - d0 + d5;
    double h = sqrt(pow(x - cos(non_relative_j0)*(d1+d4), 2) +
        pow(y - sin(non_relative_j0)*(d1+d4), 2)+
        pow(h_hight, 2));

    //Calculate angle of lower joint j1
    double non_relative_j1 = PI/2 - asin(h_hight/h) - acos((d2*d2 + h*h - d3*d3)/(2*d2*h));
    angs.j1 =  - radiansToDegrees(non_relative_j1) + this->j1_start;

    //Calculate angle of upper joint j2
    double non_relative_j2 = acos((d2*d2 + d3*d3 -h*h)/(2*d2*d3));
    angs.j2 =  radiansToDegrees(PI/2 -non_relative_j2 + non_relative_j1) + this->j2_start;
    

    ROS_INFO("j0 = %f,j1 = %f,j2 = %f", angs.j0, angs.j1, angs.j2);

    return angs;
}

void UarmController::moveArm(double x, double y, double z)
{
    AngleSetting angs = invKinematic(x, y, z);
    uarm::MoveToJoints srv;
    srv.request.j0 = angs.j0;
    srv.request.j1 = angs.j1;
    srv.request.j2 = angs.j2;
    srv.request.j3 = j3_start;
    srv.request.interpolation_type = 2;
    srv.request.movement_duration = ros::Duration(3,0);
    client.call(srv);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "uarm_control");
  ros::NodeHandle n;
  
  UarmController uc = UarmController(n);
  ros::Rate loop_rate(1);
  uc.moveArm(25,0,0);

  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }
}




