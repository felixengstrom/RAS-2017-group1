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
struct Line{
    float x1;
    float x2;
    float y1;
    float y2;
};
class WallAdder{
    private:
        float lidar_displ_x;
        float lidar_displ_y;
        float lidar_displ_omega;
        float POImaxDist;
        float POIminError;
        int tolerance;
        int minPOI;
        std::vector<Line> map;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher wall_pub;
        ros::Publisher vis_pub;
        ros::NodeHandle n;
        tf::TransformListener listener;
        void publishPOI(std::vector<float> dists);

    public:
        WallAdder(std::vector<Line> map_, float POImaxDist_,
                  float POIminError_, int tolerance_, int minPOI_);
        std::vector<float> rayTrace(float x, float y, float angle);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

};

WallAdder::WallAdder(std::vector<Line> map_,
          float POImaxDist_,
          float POIminError_,
          int tolerance_,
          int minPOI_):POImaxDist(POImaxDist_), POIminError(POIminError_),
                       tolerance(tolerance_),minPOI(minPOI_), map(map_), n(),
                       listener()
{
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("robot","laser",
                                  ros::Time(0), ros::Duration(2));
        listener.lookupTransform("robot","laser",
                                 ros::Time(0), transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("laser transform not found");
    }

    lidar_displ_x = transform.getOrigin().x();
    lidar_displ_y = transform.getOrigin().y();
    lidar_displ_omega = tf::getYaw(transform.getRotation());
    sub = n.subscribe("/scan", 1, &WallAdder::scanCallback, this);
    pub = n.advertise<sensor_msgs::LaserScan>("wall_dists", 10);
    wall_pub = n.advertise<geometry_msgs::PoseArray>("wall_add", 10);
    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "updatedMap", 0 );
}
void WallAdder::publishMap()
{
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = "map";
    wall_marker.header.stamp = ros::Time();
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
    ros::Time t = truey.header.stamp;
    try{
        listener.waitForTransform("map","robot",
                              t, ros::Duration(2));
        listener.lookupTransform("map","robot",
                             t, transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("laser transform not found in wall adder node");
        return;
    }
    float x = transform.getOrigin().x();
    float y = transform.getOrigin().y();
    float angle = tf::getYaw(transform.getRotation());

    std::vector<float> dists = rayTrace(x, y, angle);
    std::vector<int> pointsOfInterest(dists.size());
    for (int i  = 0; i< truey.ranges.size(); i++){
        double lidar_range = truey.ranges[i];
        double map_range = dists[i];
        if (lidar_range < POImaxDist and std::abs(lidar_range - map_range) > POIminError ) {
            dists[i] = truey.ranges[i];
            pointsOfInterest[i] = 1;
        }else {
            //ROS_INFO("diff %f, lidar %f, map %f, i %d",
            //         std::abs(lidar_range-map_range),  lidar_range, map_range, i);
            pointsOfInterest[i] = 0;
            dists[i] = std::numeric_limits<double>::infinity();
        }
    }
    bool counting = false;
    int first = 0; 
    int last = 0;
    int firstMax = 0;
    int lastMax = 0;
    int count = 0;
    int countMax = 0;
    int tol = tolerance;

    for(int i = 0; i<pointsOfInterest.size(); i++)
    {
        if (pointsOfInterest[i])
        {
            if (counting)
            {
                count++;
                tol = tolerance;
            }
            else
            {
                first = i;
                counting = true;
                count++;
            }
        }
        else
        {
            if (counting)
            {
                if (tol>0)
                {
                    tol--;
                } else {
                    last = i-tolerance-1;
                    if (count>countMax)
                    {
                       countMax = count;
                       lastMax = last;
                       firstMax = first;
                    }

                    count = 0;
                    counting = false;
                    tol = tolerance;
                }
            }
        }
    }
    ROS_INFO("countMax %d, firstMax %d, lastMax %d", countMax, firstMax, lastMax);
    // Calculate wall
    if (countMax>minPOI)
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
        line.y2 = y - y_disp 
                 - dists[lastMax]*sin(angle + lastMax * 2*PI/360 + lidar_displ_omega);
        map.push_back(line);

        ROS_INFO("x1 %f,y1 %f,x2 %f,y2 %f", line.x1, line.y1, line.x2, line.y2);

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

        wall_pub.publish(pa);

        publishPOI(dists);
    }

}

std::vector<float> WallAdder::rayTrace(float x, float y, float angle)
{
    int n_angles = 359;
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
                dists[a] = (x1*y2-x2*y1)/(y2-y1);
            }

        }

    }
    return dists;
}
int main(int argc, char*argv[])
{
    ros::init(argc, argv, "wall_adder");
    ros::NodeHandle nh("~");
    std::string _map_file;
    std::string line;
    nh.param<std::string>("map_file", _map_file, "lab_maze_2017.txt");

    ROS_INFO_STREAM("Loading the maze map from " << _map_file);

    std::ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
        return -1;
    }

    ros::NodeHandle n;
    std::vector<Line> map;
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
        map.push_back(wall);
        c++;
    }
    ROS_INFO_STREAM("Read "<<c<<" walls from map file.");


    float POImaxDist;
    nh.param<float>("POImaxDist", POImaxDist, 0.6);
    float POIminError;
    nh.param<float>("POIminError", POIminError, 0.1);
    int tolerance;
    nh.param<int>("tolerance", tolerance, 2);

    WallAdder wa(map, POImaxDist, POIminError, tolerance);
    ros::Rate rate(10);
    while(ros::ok())
    {
        wa.publishMap();
        ros::spinOnce();
        rate.sleep();

    }

    ros::spin();
}
