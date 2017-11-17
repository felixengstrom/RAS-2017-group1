/*
 *  world_node.cpp
 *
 *
 *  Created on: Sept 18, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2015, Rares Ambrus, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
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

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "maze_map_node");
    ros::NodeHandle n("~");
    ros::Rate r(10);


    std::string _map_file;
    std::string _map_frame = "/map";
    std::string _map_topic = "/maze_map";
    std::string _map_OG = "/maze_OccupancyGrid";
    n.param<std::string>("map_file", _map_file, "maze_map.txt");
//    n.param<string>("map_frame", _map_frame, "/map");
//    n.param<string>("map_topic", _map_topic, "/maze_map");


    std::ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    
    // Occupancy Grid publisher
    ros::Publisher OG_pub= n.advertise<nav_msgs::OccupancyGrid>( _map_OG,0);

    //Here we create Occupancy grid message
    nav_msgs::OccupancyGrid OG;

    OG.header.stamp = ros::Time();
    OG.header.frame_id= "/map";

    OG.info.map_load_time= ros::Time();

    float _res;
    n.param<float>("grid_resolution",_res, (float)0.01); 
    float _size;
    n.param<float>("maze_size",_size,(float)2.5);

    int _width_height= (int)(_size/_res);
     // assumed: square maze

    OG.info.resolution = _res;
    OG.info.width = _width_height;
    OG.info.height = _width_height;
    std::vector<int8_t> _data(_width_height*_width_height);
    OG.data=_data; // initialize free map
    
    geometry_msgs::Pose _origin;

    _origin.position.x=0; _origin.position.y=0; _origin.position.z=0;
    _origin.orientation.x=0;_origin.orientation.y=0; _origin.orientation.z=0; _origin.orientation.w=0;
    OG.info.origin=_origin;    
    //uint32_t 
    //set all 
    
    std::string line;
    int wall_id = 0; int k=0;
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
            ROS_WARN("Segment error. Skipping line: %s",line.c_str());
        }
        // angle and distance
        double angle = atan2(y2-y1,x2-x1);
        double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
	//Bresenham's Line algorithm

	int X0,Y0,X1,Y1;
	k=k+1;
	if(x2>=x1)
	{
	
		ROS_INFO_STREAM("First IF loop: "<< k);
		 X0= std::max(0,(int)(x1/_res));
		 Y0=std::max(0,(int)(y1/_res));
		 X1  = std::max(0,(int)(x2/_res));
		 Y1  = std::max(0,(int)(y2/_res));
	}
	else
	{
		 ROS_INFO_STREAM("NOT FIRST! " << k );	
		 X0= std::max(0,(int)(x2/_res));
		 Y0= std::max(0,(int)(y2/_res));
		 X1  = std::max(0,(int)(x1/_res));
		 Y1  = std::max(0,(int)(y1/_res));
	}

	ROS_INFO_STREAM("The Value of X0 is: " << X0 << " and the Y0 is: "<< Y0 );		
	ROS_INFO_STREAM("The Value of X1 is: " << X1 << " and the Y1 is: "<< Y1 );		
	int dX,dY,p,X,Y, A,B,step;
	if(Y1>=Y0)
	{
		step=1;
	}
	else
	{
		step=-1;
	}
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



	
        // set pose

        // add to array
    }
    ROS_INFO_STREAM("Read "<<wall_id<<" walls from map file.");

    // Main loop.
    while (n.ok())
    {
	OG_pub.publish(OG);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} 
