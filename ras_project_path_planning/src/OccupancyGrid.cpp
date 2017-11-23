// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
// Boost includes
#include <stdio.h>
#include <stdlib.h>



// std includes
#include <limits>
#include <iostream>
#include <fstream>

//using namespace std;


class OccupancyGrid
{
	private:
		//ROS Standard
		ros::NodeHandle n;
		ros::Publisher OG_pub; //the OccupancyGrid publisher
		ros::Subscriber wall_sub; // the subscriber that add wall to the existing Occupancy grid
		ros::Subscriber object_sub;// The subscriber that add obstacle (and object)?
		//OccupancyGrid Data
		nav_msgs::OccupancyGrid OG;
		//----------------------//
		std::string _map_file; // Name of Map file
		std::string _map_OG; // Name of publishing topic
		bool initialized;
    		//MAP Variables
		float _res;
		float _size;
		int _width_height;
		//-----------//
		//Obect add variables
		double object_size;
		double battery_x; // rectangular x > y
		double battery_y; 
		double obstacle_size;
		//------------------//

		//Method declaration
		bool Map_initialize();
		void WallCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
		void ObjectCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg);
		void ConstructWall(const double x1, const double y1, const double x2, const double y2);
	public:
		OccupancyGrid() : initialized(false), _map_OG("/maze_OccupancyGrid"),n("~")
		{
    		OG_pub= n.advertise<nav_msgs::OccupancyGrid>( _map_OG,0);
		Map_initialize();
		//Object add variable initialization
		n.param<double>("object_size",object_size,0.04);
		n.param<double>("battery_x",battery_x,0.15);
		n.param<double>("battery_y",battery_y,0.06);
		n.param<double>("obstacle_size",obstacle_size,0.1);
		wall_sub = n.subscribe("/wall_add",10,&OccupancyGrid::WallCallback,this);
		object_sub = n.subscribe("/object_add",10,&OccupancyGrid::ObjectCallback,this);
		}
		
		void loop_function();


};
bool OccupancyGrid::Map_initialize()
{


    n.param<std::string>("map_file", _map_file, "maze_map.txt");
    std::ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return false;
    }

    OG.header.stamp = ros::Time::now(); // Time() before
    OG.header.frame_id= "/map";

    OG.info.map_load_time= ros::Time::now(); // Time() before
    
// -------- Initialize map variable OG ----- //    
    //Initialize size and resolution of map
    n.param<float>("grid_resolution",_res, (float)0.01); 
    n.param<float>("maze_size",_size,(float)2.5);
    _width_height= (int)(_size/_res);
    OG.info.resolution = _res;
    OG.info.width = _width_height;
    OG.info.height = _width_height;
    std::vector<int8_t> _data(_width_height*_width_height);
    OG.data=_data; // initialize free map
    //Initialize origin of map
    geometry_msgs::Pose _origin;

    _origin.position.x=0; _origin.position.y=0; _origin.position.z=0;
    _origin.orientation.x=0;_origin.orientation.y=0; _origin.orientation.z=0; _origin.orientation.w=0;
    OG.info.origin=_origin;    
// ---------------------------------------- //
// ------------Draw Walls from original MAP file-----------//

    std::string line;
    while (getline(map_fs, line)){

        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line.c_str()); continue;
        }
        // angle and distance
		
    	ConstructWall(x1,y1,x2,y2);
    
    
    }
    initialized=true;
}


void OccupancyGrid::ConstructWall(const double x1, const double y1, const double x2, const double y2)
{
	//Bresenham's Line algorithm

	int X0,Y0,X1,Y1;
	if(x2>=x1)
	{	
		 X0= std::max(0,(int)(x1/_res));
		 Y0=std::max(0,(int)(y1/_res));
		 X1  = std::max(0,(int)(x2/_res));
		 Y1  = std::max(0,(int)(y2/_res));
	}
	else
	{
		 X0= std::max(0,(int)(x2/_res));
		 Y0= std::max(0,(int)(y2/_res));
		 X1  = std::max(0,(int)(x1/_res));
		 Y1  = std::max(0,(int)(y1/_res));
	}
	int dX,dY,p,X,Y, A,B,step;

	if(Y1>=Y0){step=1;}
	else{step=-1;}
	
	dX  = X1 -X0;
        dY  = abs(Y1 -Y0);
	A   = 2*dY;
	B   = A-2*dX;
	p   = A-dX;
	X   =  X0; Y = Y0; 
	if(dX==0)
	{
		while(Y!=Y1)
		{
			OG.data[_width_height*Y+X]=(int8_t)100;
			Y=Y+step;
		}
	}
	else if(dY==0)
	{

		while(X!=X1)
		{
			OG.data[_width_height*Y+X]=(int8_t)100;
			X=X+1;
		}
	}
	else if(dY/dX<=1)
	{
	while(X < X1)
	{
	
		if(p>=0)
		{
			OG.data[_width_height*Y+X]=(int8_t)100;
			Y=Y+step;
			p=p+B;
		}
		else
		{
			OG.data[_width_height*Y+X]=(signed char)100;
			p=p+A;
		}
		X=X+1;


	}
	}
	else
	{
		A   = 2*dX;
		B   = A - 2*dY;
		p   = A-dY;
		X   =  X0; Y = Y0; 
		while(Y != Y1)
		{
			if(p>=0)
			{
				OG.data[_width_height*Y+X]=(int8_t)100;
				X=X+1;
				p=p+B;
			}
			else
			{
				OG.data[_width_height*Y+X]=(signed char)100;
				p=p+A;
			}
			Y=Y+step;
		}
	}

}
void OccupancyGrid::WallCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	geometry_msgs::Pose adp; //add points
	double x1,y1,x2,y2;
	if(msg->poses.size()==2)
	{
		ConstructWall(msg->poses[0].position.x,msg->poses[0].position.y,msg->poses[1].position.x,msg->poses[1].position.y);
		
    		OG.info.map_load_time= ros::Time::now(); // Time() before
    		OG.header.stamp= ros::Time::now(); // Time() before
	}
	else ROS_INFO_STREAM("Wall Message recieved should be Two points!");
}
void OccupancyGrid::ObjectCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
	int ID=(int)msg->pose.orientation.x; // temporary identifier

	switch(ID){
		case 1 : break; //Object
		case 2 : break;	//Obstacle
		case 3 : break;	//Battery
		case 0 : break; //Delete object from map
		default : break;
	}
}
void OccupancyGrid::loop_function()
{
	
	OG_pub.publish(OG);
}
int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "maze_map_node");

    OccupancyGrid OG;
    ros::Rate r(10);

    // Main loop.
    while (ros::ok())
    {
	OG.loop_function();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
