//ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

//ROS MSG INCLUDE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
//C++
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <map>
#include <algorithm>



class Exploration
{
	private:
		ros::NodeHandle n;
	
		//MAP Subscribers
		ros::Subscriber OG_sub; // OccupancyGrid Subscribe
		ros::Subscriber Csp_sub; //Configuration space Subscribe
		//Current Position Subscriber
		ros::Subscriber curr_sub;
		
		//--------------//		
		//Path Publisher
		ros::Publisher Dest_pub; //Destination publisher
		//--------------------//
		//State Variable  
		ros::Time t_update;
		nav_msgs::OccupancyGrid Exp; //EXPERIENCE!
		bool Exp_initialized;
		//-------------------------//
		
		//Private methods
		void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void CspCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void CurrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		
	public:

		Exploration() : Exp_initialized(false)
		{
			//Initialize
			n = ros::NodeHandle();
			OG_sub = n.subscribe("/maze_OccupancyGrid",10,&Exploration::OGCallback,this);
			Csp_sub = n.subscribe("maze_CSpace",10,&Exploration::CspCallback,this);		
			curr_sub = n.subscribe("/robot/pose",10,&Exploration::CurrCallback,this);
			
		}

		void loop_function();

};

Exploration::OGCallback(const nav_msgs::OccupancyGrid::ConstPtr&

