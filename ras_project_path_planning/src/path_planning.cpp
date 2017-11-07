// ROS includes.
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

//C++
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <map>
#include <algorithm>

struct point {
	point() : g(std::numeric_limits<double>::max()), f(std::numeric_limits<double>::max()) {}
	int x;
	int y;
	double g;
	double f;
	};
bool cmp(const std::pair<int,point>& s1, const std::pair<int,point>& s2)
{ return s1.second.f  < s2.second.f ; }

double heuristic(const point& p1, const point& p2)
{return (double)sqrt(pow((double)(p1.x+p2.x),2)+pow((double)(p1.y+p2.y),2)); }


class PathPlanning
{
	private:

		ros::NodeHandle n;
		ros::Subscriber OG_sub;
		ros::Publisher C_pub;
		ros::Publisher Path_pub; // publish set of points
		//for updating only when dif
		ros::Time t_update;
		//map variables
		float res;
		int width_height;
		std::vector<int8_t> Csp;
		//constant
		static const double r=0.274/2; //radius of robot
		int cells; // amount of cells to be filled from a point

		//Path Planning
		std::map <int, int > cameFrom;

		//Path Smoothing
		std::vector<int> path_list;
	public:
		PathPlanning(): Csp(250*250)
		{
		n = ros::NodeHandle();
		t_update=ros::Time::now();
		OG_sub = n.subscribe("/maze_OccupancyGrid",10,&PathPlanning::OGCallback,this);
		C_pub  = n.advertise<nav_msgs::OccupancyGrid>("maze_CSpace",1000);
		}
		void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // Creates C-space Csp
		void Path(int x0, int y0, int x1,int y1); // A* evaluates path
		void Reconstruct_path(int curr_index); //Reconstruct path from Path() WITH SMOOTHING
		bool Checkline(int start, int goal); //used in Reconstruct_path, check if line between two points [start,goal] is empty
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
	return;
}


void PathPlanning::Path(int x0, int y0, int x1, int y1)
{

//A*
double rt2 = sqrt(2);
//int x,y; //Goal pos in meter
//int x0,y0; // Current Robot position from Odometry
int x_cell = (int)(x1/res);
int y_cell = (int)(y1/res);

int x_start = (int)(x0/res);
int y_start = (int)(y0/res);

std::map <int,point> openSet;
//goal pos
point goalpos;
goalpos.x = x_cell;
goalpos.y = y_cell;
//start pos
point startpos; 
startpos.x=x_start; 
startpos.y=y_start;
startpos.g=0;
startpos.f=startpos.g+heuristic(startpos,goalpos);
int tmp1=startpos.x+width_height*startpos.y;
openSet.insert(std::pair<int,point>(tmp1,startpos));
cameFrom.insert(std::pair<int,int>(tmp1,tmp1));
std::map<int,point> closeSet;
//Herustic h(s0,s1) = abs(s1-s0)
while(openSet.size()!=0)
{
	//Check for lowest f value	
	std::map<int,point>::iterator element = std::min_element(openSet.begin(),openSet.end(),cmp);
	//int D = std::distance(openSet.begin(),element); //Index of biggest f
	int D = element->first;
	point current= openSet[D];
	int curr_index = current.x+current.y*width_height;
	if(current.x==goalpos.x && current.y==goalpos.y)
	{Reconstruct_path(curr_index);return;}
	closeSet.insert(std::pair<int,point>(curr_index,current));
	openSet.erase(D);
	//
	for(int i=-1; i<=1; i++)
	{
		for(int j=-1;j<=1;j++)
		{
			if(i==0 && j==0){continue;}
			int iter_index = curr_index+j+i*width_height;
			if(Csp[iter_index]!=0){continue;}//check for wall / obstacle
			if(closeSet.find(iter_index)!=closeSet.end()){continue;}//check if already exist in closedset
			point neighbour;
			neighbour.x=current.x+j; neighbour.y=current.x+i;
			neighbour.g=current.g+sqrt(pow(i,2)+pow(j,2));
			neighbour.f=neighbour.g+heuristic(neighbour,goalpos);
			
			if(openSet.find(iter_index)==openSet.end()){openSet.insert(std::pair<int,point>(iter_index,neighbour));}//check if already in openset
			if(cameFrom.find(iter_index)==cameFrom.end()){cameFrom.insert(std::pair<int,int>(iter_index,curr_index));} //add current index to map, with its parent index as value
			if(neighbour.g >=openSet[iter_index].g){continue;} //if the already in openset have lower g (or same, for previous insertion) this path not optimal, skip
			openSet[iter_index].g=neighbour.g;
			openSet[iter_index].f=neighbour.f;
			cameFrom[iter_index]=curr_index;//update parent to the best parent! (children can choose parents, halleljuah
			

		}
	}
			


}


return;
}



void PathPlanning::Reconstruct_path(int curr_index)
{

	int child;
	int parent;
	child=curr_index;
	parent=cameFrom[child];
//	std::vector<int> reverse_path;
//	reverse_path.push_back(child);
//	reverse_path.push_back(parent);

	
	
	int smooth; //smoothing from this point to iterative close
	std::vector<int> reverse_smooth;
	reverse_smooth.push_back(child);
	smooth=child;
while(parent!=child)
{
	child = parent;
	parent = cameFrom[child];
//	reverse_path.push_back(parent);
	if(Checkline(smooth,parent)){continue;}
	reverse_smooth.push_back(child);
	smooth=child;

}
std::reverse(reverse_smooth.begin(),reverse_smooth.end());
path_list=reverse_smooth;
return;
}


bool PathPlanning::Checkline(int start, int goal)
{
	//return false if wall found, if no wall, return true
	//Bresenham's Line algorithm
	int x0,y0,x1,y1;

	x0=start%width_height;
	y0=(int)(start/width_height);
	x1=goal%width_height;
	y1=(int)(goal/width_height);
	int X0,Y0,X1,Y1;
	if(x1>=x0)
	{
	
		 X0= x0;
		 Y0= y0;
		 X1  = x1;
		 Y1  = y1;
	}
	else
	{
		 X0= x1;
		 Y0= y1;
		 X1  = x0;
		 Y1  = y0;
	}

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
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
			Y=Y+step;
		}
	}
	else if(dY==0)
	{

		while(X!=X1)
		{
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
			X=X+1;
		}
	}
	else if(dY/dX<=1)
	{
	while(X < X1)
	{
	
		if(p>=0)
		{
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
			Y=Y+step;
			p=p+B;
		}
		else
		{
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
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
				if(Csp[width_height*Y+X]!=0){return false;}//found wall
				X=X+1;
				p=p+B;
			}
			else
			{
				if(Csp[width_height*Y+X]!=0){return false;}//found wall
				p=p+A;
			}
			Y=Y+step;
	
	
		}
	}

return true;
}
int main(int argc, char **argv)
{

	ros::init(argc,argv,"path_planner");
	PathPlanning P = PathPlanning();
	ros::spin();	
	return 0;
}
