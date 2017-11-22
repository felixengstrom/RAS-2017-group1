// ROS includes.
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
//TMP ROS includes

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

struct point {
	point() : g(std::numeric_limits<double>::max()), f(std::numeric_limits<double>::max()) {}
	int x;
	int y;
	double g;
	double f;
	};
bool cmp(const std::pair<int,point>& s1, const std::pair<int,point>& s2)
{ return s1.second.f  < s2.second.f ; }



class PathPlanning
{
	private:

		ros::NodeHandle n;
		ros::Subscriber OG_sub; //subscribe to Occupany Grid, Create C-space
		 //get current position from Localization
		ros::Subscriber goal_sub; //get goal position
		ros::Subscriber curr_sub;
		ros::Publisher C_pub;
		ros::Publisher Path_pub; // publish set of points for rviz
		ros::Publisher Path_follower_pub; //publish for pathfollower
		tf::TransformListener listener;
		//for updating only when dif
		ros::Time t_update; // For OG callback
		ros::Time goal_update; // for Goal Callback
		ros::Time gNow;
		ros::Time tNow;
		//map variables
		float res;
		int width_height;
		std::vector<int8_t> Csp;
		//constant
		static const double r=0.29/2; //0.274/2; //radius of robot
		int cells; // amount of cells to be filled from a point

		//Path Planning
		std::map <int, int > cameFrom;
		double x_start, y_start,x_goal,y_goal;
		//Path Smoothing
		std::vector<int> path_list;
		//A* Heuristic wall avoiding value
		int Wall_step;
		double Wall_cost;
		int Wall_tolerance; //integer can not be bigger than Wall_step!
		double WallT;
	public:
		PathPlanning(): Csp(250*250), listener(), x_start(-1),y_start(-1), Wall_step(0), Wall_cost(0),Wall_tolerance(0)
		{
			WallT=Wall_cost*(double)Wall_tolerance; //used for pathsmoothing tolerance
		n = ros::NodeHandle();
		t_update=ros::Time::now();
		goal_update=ros::Time::now();
		curr_sub = n.subscribe("/robot/pose",10,&PathPlanning::CurrCallback,this);
		OG_sub = n.subscribe("/maze_OccupancyGrid",10,&PathPlanning::OGCallback,this);
		goal_sub= n.subscribe("/robot/goal",10,&PathPlanning::GoalCallback,this);
		C_pub  = n.advertise<nav_msgs::OccupancyGrid>("maze_CSpace",1000);
		Path_pub = n.advertise<visualization_msgs::Marker>("Path_plan_marker",0);
		Path_follower_pub = n.advertise<geometry_msgs::PoseArray>("/pose_teleop",0);
		}
		double checkwall(const int index_now);
   		double  heuristic(const point& p1, const point& p2);
		void CurrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
		{x_start=msg->pose.position.x; y_start = msg-> pose.position.y; return; }
		void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // Creates C-space Csp
		void Path(double x0, double y0, double x1,double y1); // A* evaluates path
		void Reconstruct_path(int curr_index); //Reconstruct path from Path() WITH SMOOTHING
		bool Checkline(int start, int goal); //used in Reconstruct_path, check if line between two points [start,goal] is empty
		void loop_function();
};

double PathPlanning::checkwall(const int index_now)
{
	int upper_limit = width_height*width_height;
	int checkval;
	double maxval;
	int dx,dy;
	double tot;
	maxval=0;
	for(int i=-1*Wall_step; i<=Wall_step;i++)
	{
		for(int j=-1*Wall_step; j <= Wall_step; j++)
		{
		checkval= index_now + i + j*width_height;
		if(checkval > upper_limit || checkval < 0){continue;}
		if(Csp[checkval]!=0)
			{
				dx=Wall_step-abs(i)+1;
				dy=Wall_step-abs(j)+1;
				tot=sqrt(pow((double)dx,2)+pow((double)dy,2));
				if(tot>maxval) maxval=tot;

			}

		}

	}
	return maxval*Wall_cost;
}
double PathPlanning::heuristic(const point& p1, const point& p2)
{
	int index_now=p1.x+p1.y*width_height;
	double cost;
	cost = checkwall(index_now);
	
	return (double)(cost+sqrt(pow((double)(p1.x+p2.x),2)+pow((double)(p1.y+p2.y),2))); }
void PathPlanning::loop_function()
{

if(gNow==goal_update && t_update == tNow && x_start>=0 && y_start>=0){
	if(path_list.size()==0){
	Path(x_start,y_start,x_goal,y_goal);
	}

	if(path_list.size()>0){
	ROS_INFO_STREAM("following smoothed path points were calculated (size of vector:  "<<path_list.size() << " ) ");
	double Q0,W0,Q1,W1;
	Q0=(path_list[0]%width_height)*res;
	W0=(path_list[0]/width_height)*res;
	//PoseArray
	geometry_msgs::PoseArray following_points;
	geometry_msgs::Pose arp;
	arp.position.x=Q0; arp.position.y=W0;
//	following_points.poses.push_back(arp); commented out because starting position is redundant, and fucks with path foloowing


	//wall_marker is for rviz
	visualization_msgs::Marker wall_marker;
	wall_marker.header.frame_id = "/map";
	wall_marker.header.stamp = ros::Time();
	wall_marker.ns = "world";
	wall_marker.type = visualization_msgs::Marker::LINE_STRIP;
	wall_marker.action = visualization_msgs::Marker::ADD;
	wall_marker.scale.x = 0.01;
	wall_marker.color.a = 1.0;
	wall_marker.color.r = (0.0/255.0);
	wall_marker.color.g = (255.0/255.0);
	wall_marker.color.b = (0.0/255.0);
	wall_marker.pose.position.z = 0;
	wall_marker.pose.position.x = 0;
	wall_marker.pose.position.y = 0;
	geometry_msgs::Point pnt;
	pnt.x=Q0; pnt.y=W0;
	wall_marker.points.push_back(pnt);
	for(int i=1;i<path_list.size();i++)
	{
	
		Q1=(path_list[i]%width_height)*res;
		W1=(path_list[i]/width_height)*res;
		pnt.x=Q1;pnt.y=W1;
		wall_marker.points.push_back(pnt);
		ROS_INFO_STREAM(" x = " << Q1 << " y = " << W1);

		arp.position.x=Q1; arp.position.y=W1;
		following_points.poses.push_back(arp);
	}
	following_points.header.stamp = goal_update;
	Path_pub.publish(wall_marker);
	Path_follower_pub.publish(following_points);
	}
}
}
void PathPlanning::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	gNow=msg->header.stamp;
	if(goal_update!=gNow)
	{
		goal_update=gNow;
	//tf::StampedTransform transform;
	//listener.lookupTransform("/robot","/map",ros::Time(0),transform);
	//x_start=transform.getOrigin().x();
	//y_start=transform.getOrigin().y();

	x_goal=msg->pose.position.x;
	y_goal=msg->pose.position.y;
	path_list.clear(); // New goal, reset path list computed
	ROS_INFO_STREAM("New Goal Recieved! goal x : "<< x_goal << " goal y : " << y_goal << " path_list.size : " << path_list.size());
//	Path(x_start,y_start,x_goal,y_goal);
	}


//

}
void PathPlanning::OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	
	 tNow= msg->info.map_load_time;
	if(tNow != t_update)
	{
	t_update=tNow;
	//reset Csp
	std::fill(Csp.begin(), Csp.end(), 0);
	//Reset
	path_list.clear(); // new map, new path needed (keep in mind its not a new goal)
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
	//Hardcoded Path planning goal
	//double q,w,e,r;
	//q=0.18;w=0.16;e=2.2;r=0.19;
	//Path(q,w,e,r);
	
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


void PathPlanning::Path(double x0, double y0, double x1, double y1)
{

//A*
double rt2 = sqrt(2);
//int x,y; //Goal pos in meter
//int x0,y0; // Current Robot position from Odometry
int x_cell = (int)(x1/res);
int y_cell = (int)(y1/res);

int x_Start = (int)(x0/res);
int y_Start = (int)(y0/res);
if(Csp[x_cell+y_cell*width_height]!=0)
{
	ROS_INFO_STREAM("thats a wall");
	return;
}
//Reset values
path_list.clear();
cameFrom.clear();



std::map <int,point> openSet;
//goal pos
point goalpos;
goalpos.x = x_cell;
goalpos.y = y_cell;
//start pos
point startpos; 
startpos.x=x_Start; 
startpos.y=y_Start;
startpos.g=0;
startpos.f=startpos.g+heuristic(startpos,goalpos);
int tmp1=startpos.x+width_height*startpos.y;
openSet.insert(std::pair<int,point>(tmp1,startpos));
cameFrom.insert(std::pair<int,int>(tmp1,tmp1));
std::map<int,point> closeSet;
int amount;
amount=0;
//Herustic h(s0,s1) = abs(s1-s0)
while(openSet.size()!=0)
{
	
	//Check for lowest f value	
	std::map<int,point>::iterator element = std::min_element(openSet.begin(),openSet.end(),cmp);
	//int D = std::distance(openSet.begin(),element); //Index of biggest f
	int D = element->first;
//ROS_INFO_STREAM("at least got here i= "<< amount << " D = " << D);
	point current= openSet[D];
	int curr_index = current.x+current.y*width_height;
	if(current.x==goalpos.x && current.y==goalpos.y)
	{Reconstruct_path(curr_index);return;}
	closeSet.insert(std::pair<int,point>(curr_index,current));
	openSet.erase(D);
	//ROS_INFO_STREAM("comparison: openset size: " << openSet.size() << " also curr_index: " << curr_index << " D : " << D);
	for(int i=-1; i<=1; i++)
	{
		for(int j=-1;j<=1;j++)
		{
			if(i==0 && j==0){continue;}
			int iter_index = curr_index+j+i*width_height;
		//	ROS_INFO_STREAM("iter_index = " << iter_index << "CSP OF IT = " << (int)Csp[iter_index]);
			if(iter_index < 0 || iter_index > width_height*width_height) {continue;} //out of bound
			if(Csp[iter_index]!=0){continue;}//check for wall / obstacle
			
			if(closeSet.find(iter_index)!=closeSet.end()){continue;}//check if already exist in closedset
			point neighbour;
			neighbour.x=current.x+j; neighbour.y=current.y+i;
			neighbour.g=current.g+sqrt(pow(i,2)+pow(j,2)) +checkwall(iter_index); //HERE CHECKWALL
			neighbour.f=neighbour.g+heuristic(neighbour,goalpos);
			
			if(openSet.find(iter_index)==openSet.end()){openSet.insert(std::pair<int,point>(iter_index,neighbour));}//check if already in openset
			if(cameFrom.find(iter_index)==cameFrom.end()){cameFrom.insert(std::pair<int,int>(iter_index,curr_index));} //add current index to map, with its parent index as value
			if(neighbour.g >=openSet[iter_index].g){continue;} //if the already in openset have lower g (or same, for previous insertion) this path not optimal, skip
			openSet[iter_index].g=neighbour.g;
			openSet[iter_index].f=neighbour.f;
			cameFrom[iter_index]=curr_index;//update parent to the best parent! (children can choose parents, halleljuah
			

		}
	}
	amount=amount+1;		
	//ROS_INFO_STREAM("A* iteration: " << amount << " size of openSet: " << openSet.size()<< "  D = " << D );

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
	//if(checkwall(child)<wallT){continue;}
	//else if(Checkline(smooth,parent)){continue;}
	if(Checkline(smooth,parent)){continue;}
		
	reverse_smooth.push_back(child);
	smooth=child;

}
reverse_smooth.push_back(parent);
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
			if(checkwall(width_height*Y+X)>WallT){return false;}
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
			Y=Y+step;
		}
	}
	else if(dY==0)
	{

		while(X!=X1)
		{
			if(checkwall(width_height*Y+X)>WallT){return false;}
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
			if(checkwall(width_height*Y+X)>WallT){return false;}
			if(Csp[width_height*Y+X]!=0){return false;}//found wall
			Y=Y+step;
			p=p+B;
		}
		else
		{
			if(checkwall(width_height*Y+X)>WallT){return false;}
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
				if(checkwall(width_height*Y+X)>WallT){return false;}
				if(Csp[width_height*Y+X]!=0){return false;}//found wall
				X=X+1;
				p=p+B;
			}
			else
			{
				if(checkwall(width_height*Y+X)>WallT){return false;}
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
	PathPlanning P;
	ros::Rate r(10);
	while(ros::ok())
	{
		P.loop_function();
		ros::spinOnce();
		r.sleep();
	}

	//ros::spin();	
	return 0;
}
