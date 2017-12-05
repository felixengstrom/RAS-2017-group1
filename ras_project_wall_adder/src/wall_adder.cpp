#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <random>

static const float PI = acos(-1);
// Help struct for storing walls
struct Line{
    float x1;
    float x2;
    float y1;
    float y2;
    int type;
};

class WallAdder{
    private:
        // Lidar displacement relative to robot
        float lidar_displ_x;
        float lidar_displ_y;
        float lidar_displ_omega;

        // Tunable parameters for the wall adding
        float POImaxDist;
        float POIminError;
        int tolerance;
        int minPOI;
        int minPOIremove;

        // Map file paths
        std::string map_file;
        std::string new_map_file;

        // Map
        std::vector<Line> map;

        // Ros parameters
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher wall_pub;
        ros::Publisher wall_r_pub;
        ros::Publisher vis_pub;
        tf::TransformListener listener;

        // Private methods
        void addWall(Line line);
        void removeWall(Line line);

    public:
        ros::Time t;
        void publishPOI(std::vector<float> dists);
        void loadMap();
        // Constructor
        WallAdder( std::string map_file_, std::string new_map_file_, float POImaxDist_,
                  float POIminError_, int tolerance_, int minPOI_, int minPOIremove_);
        // Public methods
        std::vector<float> rayTrace(float x, float y, float angle, std::vector<int>& wall_id);
        bool hasSubscriber();
        void publishMap();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void saveMap(std::string new_map_file);

};

WallAdder::WallAdder(std::string map_file_,
          std::string new_map_file_,
          float POImaxDist_,
          float POIminError_,
          int tolerance_,
          int minPOI_, int minPOIremove_):POImaxDist(POImaxDist_), POIminError(POIminError_),
                                          tolerance(tolerance_),minPOI(minPOI_), n(),
                                          minPOIremove(minPOIremove_), listener(),
                                          map_file(map_file_), new_map_file(new_map_file_)
{
    // Get the lidars position relative to robot
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("robot","laser", ros::Time(0), ros::Duration(2));
        listener.lookupTransform("robot","laser", ros::Time(0), transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("laser transform not found");
    }
    lidar_displ_x = transform.getOrigin().x();
    lidar_displ_y = transform.getOrigin().y();
    lidar_displ_omega = tf::getYaw(transform.getRotation());
    
    // Initiate ros parameters
    sub = n.subscribe("/scan", 1, &WallAdder::scanCallback, this);
    pub = n.advertise<sensor_msgs::LaserScan>("wall_dists", 10);

    wall_pub = n.advertise<geometry_msgs::PoseArray>("wall_add", 10);
    wall_r_pub = n.advertise<geometry_msgs::PoseArray>("wall_remove", 10);
    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "updatedMap", 10 );
}
bool WallAdder::hasSubscriber(){
    return wall_pub.getNumSubscribers()>0;
}
void WallAdder::publishMap()
{
    // Create wall marker array with proper values
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = "map";
    wall_marker.header.stamp = t ;
    wall_marker.ns = "world";
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.y = 0.01;
    wall_marker.scale.z = 0.2;
    wall_marker.color.a = 1.0;
    wall_marker.color.r = (255.0/255.0);
    wall_marker.color.g = (0.0/255.0);
    wall_marker.color.b = (0.0/255.0);
    wall_marker.pose.position.z = 0.2;

    // Calculate and and the walls to markerArray
    int wall_id = 0;
    for (int i = 0; i<map.size(); i++)
    {

        double x1 = map[i].x1;
        double y1 = map[i].y1;
        double x2 = map[i].x2;
        double y2 = map[i].y2;

        double angle = atan2(y2-y1,x2-x1);
        double dist = sqrt(pow(x1-x2,2) + pow(y1-y2,2));
        // set pose
        wall_marker.scale.x = std::max(0.01,dist);
        wall_marker.pose.position.x = (x1+x2)/2;
        wall_marker.pose.position.y = (y1+y2)/2;
        wall_marker.text="";
        tf::Quaternion quat; quat.setRPY(0.0,0.0,angle);
        tf::quaternionTFToMsg(quat, wall_marker.pose.orientation);

        // add to array
        wall_marker.id = wall_id;
        all_markers.markers.push_back(wall_marker);
        wall_id++;

    }

    // Publish array
    vis_pub.publish(all_markers);
}
void WallAdder::publishPOI(std::vector<float> dists)
{
    sensor_msgs::LaserScan fakey;
    std_msgs::Header h;
    h.frame_id = "laser";
    h.stamp = ros::Time::now();
    fakey.header = h;
    fakey.angle_min = -3.12413907051;
    fakey.angle_max = 3.12413907051;
    fakey.angle_increment = 0.0174532923847;
    fakey.time_increment =  3.82735464655e-07;
    fakey.scan_time = 0.000137402035762;
    fakey.range_min = 0.15000000596;
    fakey.range_max = 6.0;
    fakey.ranges = dists;
    pub.publish(fakey);

}
void WallAdder::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    sensor_msgs::LaserScan truey(*msg);

    tf::StampedTransform transform;
    t = truey.header.stamp;
    try{
        listener.waitForTransform("map","est_pos",
                              t, ros::Duration(2));
        listener.lookupTransform("map","est_pos",
                             t, transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("laser transform not found in wall adder node");
        return;
    }
    float x = transform.getOrigin().x();
    float y = transform.getOrigin().y();
    float angle = tf::getYaw(transform.getRotation());

    std::vector<int> wall_id(360, -1);
    std::vector<float> dists = rayTrace(x, y, angle, wall_id);
    std::vector<float> dists2(dists);
    std::vector<int> pointsOfInterest(dists.size());
    for (int i  = 0; i< truey.ranges.size(); i++){
        double lidar_range = truey.ranges[i];
        double map_range = dists[i];
        double map_range_r = dists[(i+1)%dists.size()];
        double map_range_l = dists[(i-1)%dists.size()];
        double dist_r = map_range_r - lidar_range;
        double dist_m = map_range - lidar_range;
        double dist_l = map_range_l - lidar_range;
        double to_close = std::min(std::min(dist_r, dist_m), dist_l);
        double to_far = std::max(std::max(dist_r, dist_m), dist_l);
        //ROS_INFO("lidar range: %f,to far: %f", lidar_range, to_far);
        //ROS_INFO("map_range: %f,map_range_r: %f,map_range_l: %f", map_range, map_range_r, map_range_l);
        if (lidar_range < POImaxDist and to_close > POIminError ) {
            dists2[i] = lidar_range;
            pointsOfInterest[i] = 1;
        }else if (map_range < POImaxDist and to_far < -POIminError ){
            dists2[i] = map_range;
            pointsOfInterest[i] = 2;
        }else{
            pointsOfInterest[i] = 0;
            dists2[i] = std::numeric_limits<double>::infinity();
        }
    }
    dists = dists2;
    int first = 0; 
    int last = 0;
    int firstMax = 0;
    int lastMax = 0;
    int count = 0;
    int countMax = 0;
    int tol = tolerance;
    int type = 0;
    int typeMax = 0;

    // Check for walls to add
    int j = 0;
    while( j<dists.size() or type!=0)
    {
        int i = j%dists.size();

        // If the point of interest has the same as the current type or it is
        // the first POI
        if ((pointsOfInterest[i]==type and type!=0) or (type==0 and pointsOfInterest[i]))
        {
            if (type)
            {
                count++;
                tol = tolerance;
            }
            else
            {
                first = i;
                type = pointsOfInterest[i];
                count++;
            }
        }
        else
        {
            if (type)
            {
                if (tol>0)
                {
                    tol--;
                } else {
                    last = (i-tolerance-1)%dists.size();
                    if (count>countMax)
                    {
                       countMax = count;
                       lastMax = last;
                       firstMax = first;
                       typeMax = type;
                    }

                    count = 0;
                    tol = tolerance;
                    type=0;
                    j = j-tolerance;
                }
            }
        }
        j++;
    }
    //ROS_INFO("countMax %d, firstMax %d, lastMax %d, typMax %d", countMax, firstMax, lastMax, typeMax);
    // Calculate wall
    if (countMax>minPOI and typeMax==1)
    {
        Line line;
        float x_disp = lidar_displ_x*cos(angle) - lidar_displ_y*sin(angle);
        float y_disp = lidar_displ_y*cos(angle) + lidar_displ_x*sin(angle);
        line.x1 = x + x_disp 
               - dists[firstMax]*cos(angle + firstMax * 2*PI/360 + lidar_displ_omega);
        line.y1 = y + y_disp 
               - dists[firstMax]*sin(angle + firstMax * 2*PI/360 + lidar_displ_omega);
        line.x2 = x + x_disp 
                 - dists[lastMax]*cos(angle + lastMax * 2*PI/360 + lidar_displ_omega);
        line.y2 = y + y_disp 
                 - dists[lastMax]*sin(angle + lastMax * 2*PI/360 + lidar_displ_omega);
        line.type = 1;
        map.push_back(line);
        addWall(line);

        //ROS_INFO("x1 %f,y1 %f,x2 %f,y2 %f", line.x1, line.y1, line.x2, line.y2);
    }
    
    int w_id = wall_id[firstMax];
    if (typeMax==2 and map[w_id].type == 1 and countMax>minPOIremove)
    {
        ROS_INFO("trying to remove wall");
        removeWall(map[w_id]);
        map.erase(map.begin()+w_id);
    }
    publishPOI(dists);
}
void WallAdder::addWall(Line line){

        //ROS_INFO("adding wall x1: %f,  y1: %f,  x2: %f ", line.x1, line.y1, line.x2);
        geometry_msgs::PoseArray pa;
        geometry_msgs::Pose p1;
        geometry_msgs::Pose p2;
        p1.position.x = line.x1;
        p1.position.y = line.y1;
        p2.position.x = line.x2;
        p2.position.y = line.y2;
        std::vector<geometry_msgs::Pose> poses(2);
        poses[0] = p1;
        poses[1] = p2;
        std_msgs::Header h;
        h.frame_id = "map";
        pa.header = h;
        pa.poses = poses;
        wall_pub.publish(pa);
        //ROS_INFO("added? wall x1: %f,  y1: %f,  x2: %f ", line.x1, line.y1, line.x2);
}

void WallAdder::removeWall(Line line){

        //ROS_INFO("adding wall x1: %f,  y1: %f,  x2: %f ", line.x1, line.y1, line.x2);
        geometry_msgs::PoseArray pa;
        geometry_msgs::Pose p1;
        geometry_msgs::Pose p2;
        p1.position.x = line.x1;
        p1.position.y = line.y1;
        p2.position.x = line.x2;
        p2.position.y = line.y2;
        std::vector<geometry_msgs::Pose> poses(2);
        poses[0] = p1;
        poses[1] = p2;
        std_msgs::Header h;
        h.frame_id = "map";
        h.stamp = ros::Time::now();
        pa.header = h;
        pa.poses = poses;
        wall_r_pub.publish(pa);
}


std::vector<float> WallAdder::rayTrace(float x, float y, float angle, std::vector<int>& wall_id)
{
    int n_angles = 360;
    std::vector<float> dists(n_angles);
    for(int a = 0; a<n_angles; a++)
    {
        int n_walls = map.size();
        float angle_n = angle + a*2*PI/360 + PI + lidar_displ_omega;
        float x_disp = lidar_displ_x*cos(angle) - lidar_displ_y*sin(angle);
        float y_disp = lidar_displ_y*cos(angle) + lidar_displ_x*sin(angle);
        float ca = cos(angle_n);
        float sa = sin(angle_n);
        dists[a] = 10;
        for(int w = 0; w<n_walls; w++)
        {
            float x1 = map[w].x1 - x - x_disp,
                  x2 = map[w].x2 - x - x_disp,
                  y1 = map[w].y1 - y - y_disp,
                  y2 = map[w].y2 - y - y_disp;

            float x1_new = x1*ca + y1*sa,
                  x2_new = x2*ca + y2*sa,
                  y1_new = y1*ca - x1*sa,
                  y2_new = y2*ca - x2*sa;

            x1 = x1_new;
            x2 = x2_new;
            y1 = y1_new;
            y2 = y2_new;

            if (y1*y2 < 0 and
                (((x1>0) or (x2>=0)) and((x1*y2 - x2*y1)*y2 > 0)) and
                std::min(x1, x2)<dists[a])
            {
                wall_id[a] = w;
                dists[a] = (x1*y2-x2*y1)/(y2-y1);
            }

        }

    }
    return dists;
}
void WallAdder::loadMap()
{
    std::string line;

    ROS_INFO_STREAM("Loading the maze map from " << map_file);

    std::ifstream map_fs; map_fs.open(map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<map_file<<". Please double check that the file exists. Aborting.");
    }

    int c = 0;
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
        Line wall;
        wall.x1 = x1;
        wall.x2 = x2;
        wall.y1 = y1;
        wall.y2 = y2;
        wall.type=0;
        map.push_back(wall);
        c++;
    }
    ROS_INFO_STREAM("Read "<<c<<" walls from map file.");
    map_fs.close();


    ROS_INFO_STREAM("Loading the maze map from " << new_map_file);

    map_fs.open(new_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<new_map_file<<". Please double check that the file exists. Aborting.");
    }

    c = 0;
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
        Line wall;
        wall.x1 = x1;
        wall.x2 = x2;
        wall.y1 = y1;
        wall.y2 = y2;
        wall.type=1;
        map.push_back(wall);
        addWall(wall);
        c++;
    }
    ROS_INFO_STREAM("Read "<<c<<" walls from map file.");
}
void WallAdder::saveMap(std::string new_map_file)
{
    std::ofstream out(new_map_file, std::ofstream::trunc);
    for(int i = 0 ; i < map.size();i++){
        if (map[i].type == 1)
        {
            out << map[i].x1 << " "<< map[i].y1 << " "<< map[i].x2 << " "<< map[i].y2 << std::endl;
        }
    }
}
  
int main(int argc, char*argv[])
{

    ros::init(argc, argv, "wall_adder");
    ros::NodeHandle nh("~");

    float POImaxDist;
    nh.param<float>("POImaxDist", POImaxDist, 0.8);
    float POIminError;
    nh.param<float>("POIminError", POIminError, 0.2);
    int tolerance;
    nh.param<int>("tolerance", tolerance, 5);
    int minPOI;
    nh.param<int>("minPOI", minPOI, 15);
    int minPOIremove;
    nh.param<int>("minPOIremove", minPOIremove, 10);

    std::string map_file;
    nh.param<std::string>("map_file", map_file, "lab_maze_2017.txt");

    std::string new_map_file;
    nh.param<std::string>("new_map_file", new_map_file, "lab_maze_2017_new.txt");

    WallAdder wa(map_file, new_map_file, POImaxDist, POIminError, tolerance, minPOI, minPOIremove);

    ros::Rate rate(10);
    while(ros::ok() and !wa.hasSubscriber()){
        rate.sleep();
    }
    wa.loadMap();
    while(ros::ok())
    {
        wa.publishMap();
        ros::spinOnce();
        rate.sleep();
    }
    //wa.saveMap(new_map_file);

}
