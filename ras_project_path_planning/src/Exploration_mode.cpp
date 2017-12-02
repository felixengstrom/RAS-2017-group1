//ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
//ROS MSG INCLUDE
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include<ras_project_brain/SetGoalPoint.h>

//C++
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <map>
#include <algorithm>
#include <time.h>


class Exploration
{
	private:
		ros::NodeHandle n;
	
		//MAP Subscribers
		ros::Subscriber OG_sub; // OccupancyGrid Subscribe
		ros::Subscriber Csp_sub; //Configuration space Subscribe
		//Current Position Subscriber
		ros::Subscriber curr_sub;
	        //Switch subscribe
		ros::Subscriber sw_sub;	
		//--------------//	
		//Constants//
		static const double r = 0.29/2; //radius of robot

		//---------------//	
		//Path Publisher
		ros::Publisher Dest_pub; //Destination publisher
		ros::Publisher eMap_pub; // Published explored path so far
		ros::Publisher done_pub; //Publishes True when done, false while not done
		//Service: Client
		ros::ServiceClient dest_client;
		
		//--------------------//
		//State Variable  
		ros::Time t_update;
		nav_msgs::OccupancyGrid Exp; //EXPERIENCE!
		bool Exp_initialized;
		double xNow,yNow,wNow; //Current Position (pose)
		geometry_msgs::PoseStamped goal_pos;
		ras_project_brain::SetGoalPoint Goal_SC;
		double max_x,max_y; //the maximus size of the maze , in meters
		nav_msgs::OccupancyGrid Csp;
		bool Csp_received;		
		bool GO,GO_once;
		std_msgs::Bool Done; //publish Done when done.
		int Explore_amount;
		double exploration_percentage;
		//-------------------------//
		
		//Parameters Initializeable with n.param//
		int resolution_cutting; // The amount the exploration map is cut down to, 
		double forward_exp, sideway_exp;
		//Private methods
		void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void CspCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void CurrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void SwitchCallback(const std_msgs::Bool::ConstPtr& msg)
		{  GO_once = msg->data;  return; }
		//Exploration Stratergy Function
		double calc_explored();
		void direction_search();
		void random_search();		
		void CheckDirection(const double x1, const double y1, const double x2, const double y2);
		void WallAvoidance(int wall_dist);
		void Add_WallExplored();
	public:

		Exploration() : Exp_initialized(false),GO_once(false),GO(true),Csp_received(false)
		{

			//Initialize
			n = ros::NodeHandle();
			Done.data=false;
			Csp.header.stamp = ros::Time::now();
			n.param<int>("resolution_cutting",resolution_cutting,10);
			n.param<double>("foward_margin",forward_exp,0.2);
			n.param<double>("sideway_margin",sideway_exp,0.2);
			n.param<double>("exploration_percentage",exploration_percentage,0.5);
			OG_sub = n.subscribe("/maze_OccupancyGrid",10,&Exploration::OGCallback,this);
			Csp_sub = n.subscribe("/maze_CSpace",10,&Exploration::CspCallback,this);		
			sw_sub = n.subscribe("/Exploration/Go",10,&Exploration::SwitchCallback,this);
			curr_sub = n.subscribe("/robot/pose",10,&Exploration::CurrCallback,this);
			Dest_pub = n.advertise<geometry_msgs::PoseStamped>("/robot/goal",0);	
			eMap_pub = n.advertise<nav_msgs::OccupancyGrid>("/Explored_map",0);
			done_pub = n.advertise<std_msgs::Bool>("/Explored/done",0);
			dest_client = n.serviceClient<ras_project_brain::SetGoalPoint>("set_robot_goal");
		}

		void loop_function();

};
void Exploration::Add_WallExplored()
{
int x_ratio = Csp.info.width/Exp.info.width;
int y_ratio = Csp.info.height/Exp.info.height;
int gridsize = x_ratio * y_ratio;
int mapsize = Exp.data.size();
int i,j,xExp,yExp,xCsp,yCsp;
int add; double percentage;
int amount_added = 0;
for(i = 0; i<mapsize;i++) // i is the Exp map index
{
	xExp = i % Exp.info.width;
	yExp = i / Exp.info.width;
	xCsp = xExp * x_ratio;
	yCsp = yExp * y_ratio;
	j = xCsp + yCsp*Csp.info.width;
	add = 0;
	for(int k = 0; k < y_ratio; k++)
	{
		for(int l = 0; l < x_ratio; l++)
		{
			if(j+l+k*Csp.info.width <Csp.data.size() && j+l+k*Csp.info.width >0 ) //sanity check
			{
				if(Csp.data[j+l+k*Csp.info.width] ==100)
					add++;
			}
		}
	}
		
	percentage = (((double)add)/((double)gridsize));
	if(percentage >= 0.5)
	{
		Exp.data[i] = 100;
		amount_added++;
	}

	

}
Explore_amount = amount_added;
}
void Exploration::WallAvoidance(int wall_dist)
{
	int xCsp = goal_pos.pose.position.x /Csp.info.resolution;
	int yCsp = goal_pos.pose.position.y /Csp.info.resolution;
	int index = xCsp + yCsp*Csp.info.width;
	std::vector<int>indexPoints;
	bool loop;
	loop = false;
	for(int i = -wall_dist; i <= wall_dist; i++)
	{
		for(int j = -wall_dist; j<= wall_dist; j++)
		{
			if(index+j+i*Csp.info.width < Csp.data.size() && index+j+i*Csp.info.width >=0)
			{
				if(Csp.data[index+j+i*Csp.info.width]!=0)
					loop = true; //a close wall exist;
				else
				indexPoints.push_back(index+j+i*Csp.info.width);
			}
		}
	}
	if(loop==false){return;}
	while(loop)
	{
	index = indexPoints.back(); indexPoints.pop_back();
	loop = false;
	for(int i = -wall_dist; i <= wall_dist; i++)
	{
		for(int j = -wall_dist; j <= wall_dist; j++)
		{
			if(index+j+i*Csp.info.width < Csp.data.size() && index+j+i*Csp.info.width >=0)
			{
				if(Csp.data[index+j+i*Csp.info.width]!=0)
					loop = true;
				else
				{
				if(std::find(indexPoints.begin(),indexPoints.end(),index+j+i*Csp.info.width) == indexPoints.end())
					indexPoints.push_back(index+j+i*Csp.info.width);
			
				}
			}
		}
	}
	if(indexPoints.size()>300)
	{	
		loop = false;
		ROS_INFO_STREAM("TOO MUCH!!!");
		return;
	}
	}
	xCsp = index % Csp.info.width;
	yCsp = index / Csp.info.width;
	goal_pos.pose.position.x = xCsp*Csp.info.resolution;
	goal_pos.pose.position.y = yCsp*Csp.info.resolution;
	return;
}
double Exploration::calc_explored() //This function calculate and returns the percentage of the map that have been discovered
{
	int Map_size = Exp.data.size();
	int explored; explored = 0;
	for(int i = 0; i<Map_size; i++)
	{
		if(Exp.data[i] == 100)
			explored++;

	}
	double percentage =(double)(explored- Explore_amount)/((double)(Map_size-Explore_amount));
//	ROS_INFO_STREAM("WE have explored: " << percentage);
	return percentage;
}
void Exploration::random_search() // Random search based on current explored
{
 int Map_size = Exp.data.size();
 std::vector<int> suitable_index;
 for(int i = 0; i<Map_size; i++)
 {
	if(Exp.data[i] == 0)
		suitable_index.push_back(i);


 }
 srand(time(NULL));
 int randomChoose = rand() % suitable_index.size(); //Risk of uneven distribution here...
int randomIndex = suitable_index[randomChoose];
 int x_cell = randomIndex % Exp.info.width;
 int y_cell = randomIndex / Exp.info.width;
 double x = (double)(x_cell*Exp.info.resolution);
 double y = (double)(y_cell*Exp.info.resolution);

ROS_INFO_STREAM("chosen random point: randomindex = " << randomIndex << " x_cell = " << x_cell << " y_cell = " << y_cell << " x = " << x << " y = " << y);
bool loop =true;
int xCsp = (int)(x/Csp.info.resolution);
int yCsp = (int)(y/Csp.info.resolution);
int step = 1;
//ROS_INFO_STREAM("xCsp = " << xCsp << " yCsp = " << yCsp << " resolution = " << Csp.info.resolution << " data size " << Csp.data.size());
if(xCsp+yCsp*Csp.info.width < Csp.data.size() && xCsp+yCsp*Csp.info.width >=0)
{
if(Csp.data[xCsp+yCsp*Csp.info.width] == 0)
{
	goal_pos.pose.position.x = x; goal_pos.pose.position.y = y; goal_pos.header.stamp = ros::Time::now();

	return;
}
}
while(loop)
{
for(int i = -step; i <= step; i+=2*step)
	{
		for(int j = -step; j<=step;j++)
		{
			if(xCsp+j+(yCsp+i)*Csp.info.width < Csp.data.size() && xCsp+j+(yCsp+i)*Csp.info.width >=0)
			{	
				if(Csp.data[xCsp+j+(yCsp+i)*Csp.info.width] == 0)
				{
				
					x = (xCsp+j)*Csp.info.resolution;
					y = (yCsp+i)*Csp.info.resolution;

					goal_pos.pose.position.x = x; goal_pos.pose.position.y = y; goal_pos.header.stamp = ros::Time::now();
					return;
				}
			}
			if(xCsp+i+(yCsp+j)*Csp.info.width < Csp.data.size() && xCsp+i+(yCsp+j)*Csp.info.width >=0)
			{
				if(Csp.data[xCsp+i+(yCsp+j)*Csp.info.width] == 0)
				{
					x = (xCsp+i)*Csp.info.resolution;
					y = (yCsp+j)*Csp.info.resolution;

					goal_pos.pose.position.x = x; goal_pos.pose.position.y = y; goal_pos.header.stamp = ros::Time::now();

					return;

				}

			}
		}
	}
step++; 
if(step > 100) //resample
{
srand(time(NULL));
 randomChoose = rand() % suitable_index.size(); //Risk of uneven distribution here...
randomIndex = suitable_index[randomChoose];
 x_cell = randomIndex % Exp.info.width;
 y_cell = randomIndex / Exp.info.width;
 x = (double)(x_cell*Exp.info.resolution);
 y = (double)(y_cell*Exp.info.resolution);
xCsp = (int)(x/Csp.info.resolution);
yCsp = (int)(y/Csp.info.resolution);
step = 1;

} //for now

}
return;



}
void Exploration::direction_search()
{
double explored = calc_explored();
int x_grid = (int)(xNow/Exp.info.resolution);
int y_grid = yNow;
std::vector<double> directions;
double PI =std::acos(-1);
int num_direction = 10;
double angle;

for(int i = 0; i < num_direction; i++)
{
	angle = (double) (i * ( (2*PI)/num_direction));
	bool loop = true;
	double cos_rad = std::cos(angle);
	double sin_rad = std::sin(angle);
	int steps; steps = 0;

	
	directions.push_back(angle);

}

}


void Exploration::CheckDirection(const double x1, const double y1, const double x2, const double y2)
{
//Bresenham's Line algorithm
int map_size = Exp.data.size();
int explored; explored = 0;
int X0,Y0,X1,Y1;
if(x2>=x1)
{	
	 X0= std::max(0,(int)(x1/Exp.info.resolution));
	 Y0=std::max(0,(int)(y1/Exp.info.resolution));
	 X1  = std::max(0,(int)(x2/Exp.info.resolution));
	 Y1  = std::max(0,(int)(y2/Exp.info.resolution));
}
else
{
	 X0= std::max(0,(int)(x2/Exp.info.resolution));
	 Y0= std::max(0,(int)(y2/Exp.info.resolution));
	 X1  = std::max(0,(int)(x1/Exp.info.resolution));
	 Y1  = std::max(0,(int)(y1/Exp.info.resolution));
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
		if(Exp.info.width*Y+X <map_size)
		{
		if(Exp.data[Exp.info.width*Y+X]==0)
			explored++;
		}
		Y=Y+step;
		
	}
}
else if(dY==0)
{

	while(X!=X1)
	{
		if(Exp.info.width*Y+X <map_size)
		{
		if(Exp.data[Exp.info.width*Y+X]==0)
			explored++;
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
		if(Exp.info.width*Y+X <map_size)
		{
		if(Exp.data[Exp.info.width*Y+X]==0)
			explored++;
		}
		Y=Y+step;
		p=p+B;
	}
	else
	{
		if(Exp.info.width*Y+X <map_size)
		if(Exp.data[Exp.info.width*Y+X]==0)
			explored++;
		
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
			if(Exp.info.width*Y+X <map_size)
			if(Exp.data[Exp.info.width*Y+X]==0)
				explored++;
			
			X=X+1;
			p=p+B;
		}
		else
		{
			if(Exp.info.width*Y+X <map_size)
			if(Exp.data[Exp.info.width*Y+X]==0)
				explored++;
			
			p=p+A;
		}
		Y=Y+step;
	}
}
}


void Exploration::loop_function()
{
if(Exp_initialized==true)
{
	eMap_pub.publish(Exp);
	double percentage_explored = calc_explored();
	if((percentage_explored) >exploration_percentage)
	{
		Done.data = true;
		ROS_INFO_STREAM("We have now explored the amount specified. Done message is now true");
	}
	else
	{
		Done.data = false;
	}
	if(GO && GO_once && Csp_received && Done.data == false) //Initial random search, when initialized, GO=false makes sure its only run once
	{
		random_search();
		GO=false;
	}

	else if( sqrt(pow(xNow-goal_pos.pose.position.x,2)+pow(yNow - goal_pos.pose.position.y,2)) <0.2 && Csp_received) //New random goal generated when sufficiently close to the previous one
	{ random_search();ROS_INFO_STREAM("New random path, explored: " <<percentage_explored); }
	if(GO_once && Done.data == false)
	{
		//Goal_SC.request.goalPoint.header = goal_pos.header;
		//Goal_SC.request.goalPoint.point = goal_pos.pose.position;
		//dest_client.call(Goal_SC);
		WallAvoidance(2); // Recheck the goal_pos to be input [cm] from wall
		Dest_pub.publish(goal_pos);
	}
	
	done_pub.publish(Done);
}
return;
}
void Exploration::CspCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
if(Exp_initialized==false)
	return;
if(msg->header.stamp != Csp.header.stamp)
{
 Csp = *msg;
 if(Csp_received==false)
 {	 
	 Add_WallExplored();
 }
 Csp_received = true;
}
}
void Exploration::OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
if(Exp_initialized==false)
{
	float res,width,height;
	res=msg->info.resolution; width =  msg->info.width; height = msg->info.height;
	
	max_x = (double)(res*width); max_y = (double)(res*height);
	
	Exp.info.resolution = resolution_cutting*res;
	Exp.info.width = (int)(max_x/Exp.info.resolution);
	Exp.info.height = (int)(max_y/Exp.info.resolution);
	Exp.data.assign(Exp.info.width*Exp.info.height,0);

	geometry_msgs::Pose _origin;
	_origin.position.x=0;_origin.position.y=0;_origin.position.z=0;
	_origin.orientation.x=0;_origin.orientation.y=0;_origin.orientation.z=0;_origin.orientation.w=0;
	Exp.info.origin = _origin;
	Exp.header.stamp = ros::Time::now();
	Exp.header.frame_id = "/map";
	Exp.info.map_load_time = ros::Time::now();
	Exp_initialized = true;

}

}
void Exploration::CurrCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//Read current positio in to class variables
xNow = msg->pose.position.x; yNow = msg->pose.position.y;
tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
	wNow = yaw;
	// --------------------------------------//
	// Using current pose, compute local grid position in int index
	int x,y;
	x = (int)(xNow/Exp.info.resolution); y = (int)(yNow/Exp.info.resolution);
	//Paint its own position //
	int dist = (int) (r/Exp.info.resolution);
	for(int i = -dist ; i <= dist; i++)
	{
		for(int j = -dist; j<=dist; j++)
		{
			if(x+j+(y+i)*Exp.info.width>=0 && x+j+(y+i)*Exp.info.width < Exp.info.width*Exp.info.height)
			Exp.data[(x+j)+(y+i)*Exp.info.width]=100;
		}

	}

	//Paint in front of the robot//
	x = (int)((xNow+(r+sideway_exp/2)*std::cos(wNow))/Exp.info.resolution);
	y = (int) ((yNow + (r+ forward_exp/2)*std::sin(wNow))/Exp.info.resolution);
	int distX = (int)( (sideway_exp/2)/Exp.info.resolution);
	int distY = (int)( (forward_exp/2)/Exp.info.resolution);

	for(int i = -distX ; i <= distX; i++)
	{
		for(int j = -distY; j<=distY; j++)
		{
			if(x+j+(y+i)*Exp.info.width>=0 && x+j+(y+i)*Exp.info.width < Exp.info.width*Exp.info.height)
			Exp.data[(x+j)+(y+i)*Exp.info.width]=100;
		}

	}

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"Exploration");
	Exploration E;
	ros::Rate r(10);
	while(ros::ok())
	{
		E.loop_function();
		ros::spinOnce();
		r.sleep();
	}
	return 0 ;

}
