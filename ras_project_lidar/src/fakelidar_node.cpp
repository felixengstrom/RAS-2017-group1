#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
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

class FakeLidar
{
    private:
        std::default_random_engine rng;
        std::normal_distribution<double> dist_noise;
        std::vector<Line> map;
        ros::Subscriber truepose;
        ros::NodeHandle n;
        tf::TransformListener listener;
        
    public:
        sensor_msgs::LaserScan fakeScan();
        sensor_msgs::PointCloud particles;
        std::vector<float> rayTrace();
        FakeLidar(std::vector<Line> _map);
};

FakeLidar::FakeLidar(std::vector<Line> _map): listener(), n(), dist_noise(0,0.01), map(_map)
{
}

sensor_msgs::LaserScan FakeLidar::fakeScan()
{
    sensor_msgs::LaserScan fakey;
    std_msgs::Header h;
    h.frame_id = "laser";
    h.stamp = ros::Time();
    fakey.header = h;
    fakey.angle_min = -3.12413907051;
    fakey.angle_max = 3.12413907051;
    fakey.angle_increment = 0.0174532923847;
    fakey.time_increment =  3.82735464655e-07;
    fakey.scan_time = 0.000137402035762;
    fakey.range_min = 0.15000000596;
    fakey.range_max = 6.0;
    std::vector<float> dists = rayTrace();
    fakey.ranges = dists;

    /*
      seq: 3176
                 stamp: 
                     secs: 1509613003
                               nsecs: 156990843
                                 frame_id: laser
                                 angle_min: -3.12413907051
                                 angle_max: 3.14159274101
                                 angle_increment: 0.0174532923847
                                 time_increment: 3.82735464655e-07
                                 scan_time: 0.000137402035762
                                 range_min: 0.15000000596
                                 range_max: 6.0
    */ 
    return fakey;

}

std::vector<float> FakeLidar::rayTrace()
{   
    tf::StampedTransform transform;
    ros::Time t(0);
    listener.waitForTransform( "map","truepos",t, ros::Duration(1.0));
    listener.lookupTransform("map","truepos", t, transform);
    float x = transform.getOrigin().x();
    float y = transform.getOrigin().y();
    float angle = tf::getYaw(transform.getRotation());

    int n_angles= 360;
    std::vector<float> dists(n_angles);
    float angle_increment = 0.0174532923847;
    for(int a = 0; a<n_angles; a++)
    {
        int n_walls = map.size();
        float angle_n = angle + angle_increment*a-PI;
        float ca = cos(angle_n);
        float sa = sin(angle_n);
        dists[a] = 10;
        for(int w = 0; w<n_walls; w++)
        {
            float x1 = map[w].x1 - x,
                  x2 = map[w].x2 - x,
                  y1 = map[w].y1 - y,
                  y2 = map[w].y2 - y;

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
                dists[a] = (x1*y2-x2*y1)/(y2-y1) + dist_noise(rng);
            }
        }
        if (dists[a]>6 or rand()%20 == 0){
            dists[a] = std::numeric_limits<double>::infinity();;
                    
        }

    }
    return dists;
}

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "fakelidar_node");
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

    ros::NodeHandle n;
    geometry_msgs::PoseStamped _currentPose;

    float x_start = 0.22;
    float y_start = 0.22;

    float omega_start = PI/2;

    std_msgs::Header h;
    h.frame_id= "robot";
    h.stamp= ros::Time();

    ros::Rate rate(10);
    FakeLidar fl(map);

    ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 10);

    while( ros::ok() )
    {
        try
        {
            pub.publish(fl.fakeScan());
        }
        catch (tf::TransformException& ex)
        {
            ROS_INFO("failed to make transform");
        }
        ros::spinOnce();
        rate.sleep();

    }
}
