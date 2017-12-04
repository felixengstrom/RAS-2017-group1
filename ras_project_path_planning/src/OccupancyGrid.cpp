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
		//----------------------//
		std::string _map_file; // Name of Map file
		std::string _map_OG; // Name of publishing topic
		bool initialized;
    		//MAP Variables
		float _res;
		int _width,_height;
		std::vector<int8_t> data;
		std::vector<int8_t> data_object;
    		geometry_msgs::Pose _origin;
		ros::Time stamp, map_load_time;
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
		void add_object(float x, float y, float radius);
		void delete_object(float x, float y, float radius);
	public:
		OccupancyGrid() : initialized(false), _map_OG("/maze_OccupancyGrid"),n("~")
		{
		Map_initialize();
		//Object add variable initialization
		n.param<double>("object_size",object_size,0.04);
		n.param<double>("battery_x",battery_x,0.15);
		n.param<double>("battery_y",battery_y,0.06);
		n.param<double>("obstacle_size",obstacle_size,0.1);
    		OG_pub= n.advertise<nav_msgs::OccupancyGrid>( _map_OG,0);
		wall_sub = n.subscribe("/wall_add",10,&OccupancyGrid::WallCallback,this);
		object_sub = n.subscribe("/object_add",10,&OccupancyGrid::ObjectCallback,this);
		}
		
		void loop_function();


};
bool OccupancyGrid::Map_initialize()
{
if(initialized==true) return true;

    n.param<std::string>("map_file", _map_file, "maze_map.txt");
    std::ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return false;
    }

    stamp = ros::Time::now(); // Time() before
    map_load_time= ros::Time::now(); // Time() before
    
// -------- Initialize map variable OG ----- //    
    //Initialize size and resolution of map
    n.param<float>("grid_resolution",_res, (float)0.01); 
    std::string line1;
    double mw,mh;
    mw=0;mh=0;
    while (getline(map_fs, line1)){

        if (line1[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line1);

        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || ( x2 == max_num) || (y1 == max_num) || (y2 == max_num)){
            ROS_WARN("Segment error. Skipping line: %s",line1.c_str()); continue;
        }
    	if(x1 != max_num && x1 > mw) mw = x1;
	if(x2 != max_num && x2 > mw) mw = x2;
	
    	if(y1 != max_num && y1 > mw) mh = y1;
	if(y2 != max_num && y2 > mw) mh = y2;
    }
	mw= mw + (double)_res;
	mh= mh + (double) _res;
    
    _width = (int)(mw/_res);
    _height = (int)(mh/_res);
    data.assign(_width*_height,0);
    data_object.assign(_width*_height,0);
    //Initialize origin of map
     _origin;

    _origin.position.x=0; _origin.position.y=0; _origin.position.z=0;
    _origin.orientation.x=0;_origin.orientation.y=0; _origin.orientation.z=0; _origin.orientation.w=0;
// ---------------------------------------- //
// ------------Draw Walls from original MAP file-----------//
    map_fs.clear();
    map_fs.seekg(0,std::ios::beg);
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
    map_fs.close();
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
			if(_width*Y+X <data.size())
			{
			data[_width*Y+X]=(int8_t)100;
			}
			Y=Y+step;
			
		}
	}
	else if(dY==0)
	{

		while(X!=X1)
		{
			if(_width*Y+X <data.size())
			{
			data[_width*Y+X]=(int8_t)100;
			}
			X=X+1;
		}	
	}
	else if(dY/dX<=1)
	{
	while(X < X1)
	{
	
		if(p>=0)
		{
			if(_width*Y+X <data.size())
			{
			data[_width*Y+X]=(int8_t)100;
			}
			Y=Y+step;
			p=p+B;
		}
		else
		{
			if(_width*Y+X <data.size())
			data[_width*Y+X]=(signed char)100;
			
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
				if(_width*Y+X <data.size())
				data[_width*Y+X]=(int8_t)100;
				
				X=X+1;
				p=p+B;
			}
			else
			{
				if(_width*Y+X <data.size())
				data[_width*Y+X]=(signed char)100;
				
				p=p+A;
			}
			Y=Y+step;
		}
	}
}
void OccupancyGrid::add_object(float x, float y, float radius)
{
	float PI = std::acos(-1);
	for (float i = 0; i < 2*PI; i += PI/180.0)
	{
		int x_temp = std::max(0,(int) (x + radius*std::cos(radius)))/_res;
		int y_temp = std::max(0,(int) (y + radius*std::sin(radius)))/_res;
		data_object[_width*y_temp+x_temp] =(int8_t)100;
	}
	stamp = ros::Time::now();
    	map_load_time= ros::Time::now();
}
void OccupancyGrid::delete_object(float x, float y, float radius)
{
	float PI = std::acos(-1);
	for (float i = 0; i < 2*PI; i += PI/180.0)
	{
		int x_temp = std::max(0,(int) (x + radius*std::cos(radius)))/_res;
		int y_temp = std::max(0,(int) (y + radius*std::sin(radius)))/_res;
		data_object[_width*y_temp+x_temp] =(int8_t)0;
	}
	stamp = ros::Time::now();
    	map_load_time= ros::Time::now();
}
void OccupancyGrid::WallCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
	geometry_msgs::Pose adp; //add points
	double x1,y1,x2,y2;
	if(msg->poses.size()==2)
	{
		ConstructWall(msg->poses[0].position.x,msg->poses[0].position.y,msg->poses[1].position.x,msg->poses[1].position.y);
		
		stamp = ros::Time::now();
    		map_load_time= ros::Time::now(); // Time() before
	}
	else ROS_INFO_STREAM("Wall Message recieved should be Two points!");
}
void OccupancyGrid::ObjectCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg)
{
	int ID=(int)msg->pose.position.x; // temporary identifier
	float radius;
	switch(ID){
		case 1 : //Add object to map
			radius = object_size;
			add_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		case -1 : //Delete object from map
			radius = object_size;
			delete_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		case 2 : //Add obstacle to map
			radius = obstacle_size;
			add_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		case -2 : //Delete obstacle from map
			radius = obstacle_size;
			delete_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		case 3 : //Add battery to map
			radius = battery_x;
			add_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		case -3 : //Delete Battery from map (should never arrive)
			radius = battery_x;
			delete_object(msg->pose.position.x, msg->pose.position.y, radius);
			break;
		default : break;
	}
}
void OccupancyGrid::loop_function()
{
	if(initialized)
	{

	nav_msgs::OccupancyGrid OG;
    	OG.info.resolution = _res;
    	OG.info.width = _width;
    	OG.info.height = _height;
    	OG.info.origin=_origin;    
	std::vector<int8_t> data_send;
	data_send.assign(_width*_height,0);
	for (int i = 0; i < _width*_height; i++)
	{
		data_send[i] = data[i] | data_object[i];
	}
    	OG.data=data_send; // initialize free map
    	OG.header.stamp = stamp; // Time() before
    	OG.header.frame_id= "/map";
    	OG.info.map_load_time= map_load_time; // Time() before
	OG_pub.publish(OG);

	}
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
