#include <math.h>
#include <ros/ros.h>
#include "uarm/Pump.h"
#include <std_msgs/Bool.h>
#include "uarm/MoveToJoints.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/PointStamped.h>
#include "ras_project_uarm/MoveArmCartesian.h"

static const double PI = acos(-1);
static const double ROBOT_RADIUS = 0.20;
static const double ROBOT_HEIGHT = 0.05;

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
        std::string arm_frame;
        tf::TransformListener tf_;
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Subscriber sub2;
        ros::ServiceClient client;
        ros::ServiceClient client2;
        ros::ServiceServer service;
    
    public:
        UarmController();
        bool moveArm(double x, double y, double z);
        void moveToPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        bool moveToPointService(ras_project_uarm::MoveArmCartesian::Request &req, 
                                ras_project_uarm::MoveArmCartesian::Response &res);
        void engageSuctionCallback(const std_msgs::Bool::ConstPtr& msg);
        AngleSetting invKinematic(double x,double y,double z);
};

UarmController::UarmController() : tf_()
{

    ros::NodeHandle n_("~");

    j0_start = 80;
    j1_start = 80;
    j2_start = -27;
    j3_start  = 80;

    n_.getParam("j0_start", j0_start);
    n_.getParam("j1_start", j1_start);
    n_.getParam("j2_start", j2_start);
    n_.getParam("j3_start", j3_start);

    
    d0 = 0.1;
    d1 = 0.02;
    d2 = 0.15;
    d3 = 0.16;
    d4 = 0.035;
    d5 = 0.06;

    n_.getParam("d0", d0);
    n_.getParam("d1", d1);
    n_.getParam("d2", d2);
    n_.getParam("d3", d3);
    n_.getParam("d4", d4);
    n_.getParam("d5", d5);
    
    arm_frame = "uarm_base";
    
    n_.getParam("frame", arm_frame);

    service = n.advertiseService("/uarm/moveToPose", &UarmController::moveToPointService, this);

    client = n.serviceClient<uarm::MoveToJoints>("/uarm/move_to_joints");
    client2 = n.serviceClient<uarm::Pump>("/uarm/pump");

    sub = n.subscribe("uarm/moveToPose",1, &UarmController::moveToPointCallback, this);
    sub2 = n.subscribe("uarm/engageSuction",1, &UarmController::engageSuctionCallback, this);
}

void UarmController::moveToPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::PointStamped point_robot_frame;

    try
    {
        tf_.transformPoint(arm_frame, *msg, point_robot_frame);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_INFO("Could not get transformation to perform moveToPointCallback");
    }
    float x = point_robot_frame.point.x;
    float y = point_robot_frame.point.y;
    float z = point_robot_frame.point.z;
    ROS_INFO("x:%f, y:%f, z:%f", x, y, z);
    
    if (pow(x,2) + pow(y,2) < pow(ROBOT_RADIUS, 2) && z<ROBOT_HEIGHT)
    {
        ROS_INFO("Move not allowed, to close to robot");
    } else
    {
        this->moveArm(x,y,z);
    }
}
bool UarmController::moveToPointService(ras_project_uarm::MoveArmCartesian::Request &req, 
                        ras_project_uarm::MoveArmCartesian::Response &res)
{
    bool success = false;
    /*geometry_msgs::PointStamped point_robot_frame;

    try
    {
        tf_.transformPoint(arm_frame, req.point, point_robot_frame);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_INFO("Could not get transformation to perform moveToPointCallback");
        res.error = not(success);
        return success;
    }*/

    float x = req.point.point.x;
    float y = req.point.point.y;
    float z = req.point.point.z;

    ROS_INFO("x:%f, y:%f, z:%f", x, y, z);
    

    if (pow(x,2) + pow(y,2) < pow(ROBOT_RADIUS, 2) && z<ROBOT_HEIGHT)
    {
        ROS_INFO("Move not allowed, to close to robot");
    } else
    {
        bool success =this->moveArm(x,y,z);
    }

    res.error = not(success);
    return success;
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
    angs.j0 =  radiansToDegrees(non_relative_j0) + this->j0_start;

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

void UarmController::engageSuctionCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == 1)
    {
        uarm::Pump srv;
        srv.request.pump_status = 1;
        client2.call(srv);
    } else if (msg->data == 0) {
        uarm::Pump srv;
        srv.request.pump_status = 0;
        client2.call(srv);
    }

}

bool UarmController::moveArm(double x, double y, double z)
{
    AngleSetting angs = invKinematic(x, y, z);
    uarm::MoveToJoints srv;
    srv.request.j0 = angs.j0;
    srv.request.j1 = angs.j1;
    srv.request.j2 = angs.j2;
    srv.request.j3 = j3_start;
    srv.request.interpolation_type = 2;
    srv.request.movement_duration = ros::Duration(2,0);
    bool success = client.call(srv);
    ROS_INFO("call made to uarm service");
    return success;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "uarm_control");
  
  UarmController uc;
  ros::NodeHandle nh("~");
  
  bool waitForArm = false;

  nh.getParam("waitForArm", waitForArm);

  double x_start = 0.15;
  double y_start = 0.0;
  double z_start = 0.15;

  nh.getParam("x_start", x_start);
  nh.getParam("y_start", y_start);
  nh.getParam("z_start", z_start);

  bool uarm_started = ros::service::waitForService("/uarm/move_to_joints", 5000);
  if (uarm_started){
    uc.moveArm(x_start,y_start,z_start);
  } else { 
      ROS_INFO("No uarm service running");
  }

  ros::Rate loop_rate(1);
  while(ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
   }
}
