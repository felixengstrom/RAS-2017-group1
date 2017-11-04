// ROS includes.
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

//C++
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

class PathPlanning
{
	private:

		ros::NodeHandle n;
		
		ros::Subscriber OG_sub;
		ros::Publisher C_pub;
		//for updating only when dif
		ros::Time t_update;
		//map variables
		float res;
		int width_height;
		std::vector<int8_t> Csp;
		//constant
		static const double r=0.274/2; //radius of robot
		int cells; // amount of cells to be filled from a point
	public:
		PathPlanning(): Csp(250*250)
		{
		n = ros::NodeHandle();
		t_update=ros::Time::now();
		OG_sub = n.subscribe("/maze_OccupancyGrid",10,&PathPlanning::OGCallback,this);
		C_pub  = n.advertise<nav_msgs::OccupancyGrid>("maze_CSpace",1000);
		}
		void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void Path();
};
void PathPlanning::OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	
	ros::Time now= msg->info.map_load_time;
	if(now != t_update)
	{
	t_update=now;
	std::vector<int8_t>OG=msg->data;
	res=msg->info.resolution;
	width_height=msg->info.width;
	//if(Csp.size() != width_height*width_height){ Csp.resize(width_height*width_height);}
	
	//Cspace creation
	cells=(int)(r/res);
	int i,y,x;
	for(i=0;i<OG.size();i++)
		{
		//ROS_INFO_STREAM("for i = " << i << " we have OG " << (int)OG[i] );
		if((int)OG[i]==100)//thicken points/wall
			{
				Csp[i]=100;
				//ROS_INFO_STREAM("for i = " << i << " we have Csp " << (int)Csp[i]<< " cell " << cells );
				for(y=0;y<=cells;y++)
				{
					for(x=0;x<=cells; x++)
					{
						//X forward Y up
						if(i+x+y*width_height<(width_height*width_height))
						{Csp[i+x+y*width_height]=100;}
						//ROS_INFO_STREAM("I got here when x = " << x << " y = " << y );	}
						//X backward Y up
						if(i-x+y*width_height<(width_height*width_height) && i-x+y*width_height>=0)
						{Csp[i-x+y*width_height]=100;}
						 
						//X forward Y down
						if(i+x-y*width_height<width_height*width_height && i+x-y*width_height>=0)
						{Csp[i+x-y*width_height]=100;}
						
						///X backward Y down
						if(i-x-y*width_height>=0)
						{Csp[i-x-y*width_height]=100;}
					}
				}
			}
		
		}
	}

	nav_msgs::OccupancyGrid C_space;
	C_space.info.resolution=res;
	C_space.info.width = width_height;
	C_space.info.height= width_height;
	C_space.info.map_load_time = t_update;
	C_space.info.origin=msg->info.origin;
	C_space.data=Csp;
	C_pub.publish(C_space);
}
void PathPlanning::Path()
{
	

return;
}
int main(int argc, char **argv)
{

	ros::init(argc,argv,"path_planner");
	PathPlanning P = PathPlanning();
	ros::spin();	
	return 0;
}
