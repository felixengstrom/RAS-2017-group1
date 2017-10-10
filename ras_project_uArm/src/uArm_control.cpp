#include <ros/ros.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "uArm_control");
  ros::NodeHandle nh("~");
  int j0_start,  j1_start,   j2_start, j3_start ;
  j0_start = j1_start =  j2_start = j3_start  = 0;
  nh.getParam("j0_start", j0_start);
  nh.getParam("j1_start", j1_start);
  nh.getParam("j2_start", j2_start);
  nh.getParam("j3_start", j3_start);

  while(ros::ok()){
      ROS_INFO(" j0_start %i, j1_start %i, j2_start %i, j3_start %i", j0_start, j1_start, j2_start, j3_start);
      ros::spinOnce();
   }
}


