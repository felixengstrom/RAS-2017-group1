#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
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

class ParticleFilter
{
    private:
        int nParticles;
        std::default_random_engine rng;
        std::normal_distribution<double> vel_noise;
        std::normal_distribution<double> ang_noise;
        std::vector<Line> map;

        void initiateParticles(geometry_msgs::Point initialPoint,
                               float initualAng,
                               int nParticles);
        sensor_msgs::LaserScan fakeScan(const geometry_msgs::Point32 &p,float angle, int n_rays);
        
    public:
        sensor_msgs::PointCloud particles;
        std::vector<float> rayTrace(const geometry_msgs::Point32 &pos,
                                    float angle,
                                    const std::vector<float> &angles);
        ParticleFilter(geometry_msgs::Point& initialPoint,
                       float initialAng,
                       int _nParticles,
                       std::vector<Line> _map);
        void update_particles_position(float vt, float wt);
        void update_particles_weight();
};

ParticleFilter::ParticleFilter(geometry_msgs::Point& initialPoint,
                               float initialAng,
                               int _nParticles,
                               std::vector<Line> _map):nParticles(_nParticles), vel_noise(0,0.05),
                                                       ang_noise(0,0.1), particles(), map(_map)
{   
    initiateParticles(initialPoint, initialAng, nParticles);
}

void ParticleFilter::initiateParticles(geometry_msgs::Point initialPoint,
                                       float initialAng,
                                       int nParticles)
{
    std::vector<geometry_msgs::Point32> ps(nParticles); 
    std::vector<sensor_msgs::ChannelFloat32> ch(nParticles);

    for ( int i = 0; i < nParticles; i++)
    {
        geometry_msgs::Point32 p;
        p.x = initialPoint.x +(float) vel_noise(rng);
        p.y = initialPoint.y + (float)vel_noise(rng);
        p.z = 0;
        ps[i] = p;
        std::vector<float> values(2);
        values[0] = initialAng + ang_noise(rng);
        values[1] = 1;
        ch[i].values = values;
    }

    particles.header.frame_id = "map"; 
    particles.header.stamp = ros::Time();
    particles.points = ps;
    particles.channels = ch;
}

void ParticleFilter::update_particles_position(float vt, float wt)
{
   
    for ( int i = 0; i < nParticles; i++)
    {
        
        geometry_msgs::Point32 p = particles.points[i];
        float omega_n = particles.channels[i].values[0]+ wt + ang_noise(rng);
        omega_n = omega_n-(std::round(omega_n/(2*PI))*2*PI); //OPTIMATION COULD BE MADE BY REMOVING THIS
        particles.channels[i].values[0] = omega_n;
        particles.points[i].x += (vt + (float) vel_noise(rng))*cos(omega_n);
        particles.points[i].y += (vt + (float)vel_noise(rng))*sin(omega_n);
    }

    particles.header.stamp = ros::Time();
}

void ParticleFilter::update_particles_weight()
{
    for (int i=0; i<1000;i++){
        geometry_msgs::Point32 p = particles.points[i];
        float angle = particles.channels[i].values[0];
        std::vector<float> angles(360);
        for (int j=0; j<360; j++)
        {
            angles[j]=j*2*PI/360;
        }
        std::vector<float> dists = rayTrace(p, angle, angles);
    }
   
}

sensor_msgs::LaserScan ParticleFilter::fakeScan(const geometry_msgs::Point32 &p, float angle, int n_rays)
{
    sensor_msgs::LaserScan fakey;
    std_msgs::Header h;
    h.frame_id = "lidar";
    fakey.angle_min = -3.12413907051;
    fakey.angle_max = 3.12413907051;
    fakey.angle_increment = 0.0174532923847;
    fakey.time_increment =  3.82735464655e-07;
    fakey.scan_time = 0.000137402035762;
    fakey.range_min = 0.15000000596;
    fakey.range_max = 6.0;
    std::vector<float> angles(360);
    for (int i = 0; i<360; i++)
    {
        angles[i] = i*2*PI/360;

    }
    //float32[] intensities

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

}

std::vector<float> ParticleFilter::rayTrace(const geometry_msgs::Point32 &pos, float angle, const std::vector<float> &angles)
{   
    int n_angles= angles.size();
    std::vector<float> dists(n_angles);
    for(int a = 0; a<n_angles; a++)
    {
        int n_walls = map.size();
        float angle_n = angle + angles[a];
        float ca = cos(angle_n);
        float sa = sin(angle_n);
        dists[a] = 10;
        for(int w = 0; w<n_walls; w++)
        {
            float x1 = map[w].x1 - pos.x,
                  x2 = map[w].x2 - pos.x,
                  y1 = map[w].y1 - pos.y,
                  y2 = map[w].y2 - pos.y;

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
    ros::init(argc, argv, "localisation_node");
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

    

    ros::Rate rate(2);
    ros::NodeHandle n;
    ros::Publisher parts = n.advertise<sensor_msgs::PointCloud>("particles", 100);
    ros::Publisher truepub = n.advertise<geometry_msgs::PoseStamped>("truepos", 100);
    geometry_msgs::PoseStamped pp;
    std_msgs::Header h;
    h.frame_id = "map";
    pp.header = h;
    geometry_msgs::Point p;
    p.x = 0.22;
    p.y = 0.22;
    pp.pose.position = p;
    geometry_msgs::Quaternion q;
    float alpha = PI/2;
    float w = -0.05;
    float v = 0.1;
    q.w = cos(alpha);
    q.z = sin(alpha);
    pp.pose.orientation = q;
    ParticleFilter pf(p, alpha, 1000, map);   
    while (ros::ok()){
        p.x = p.x + cos(alpha)*v;
        p.y = p.y + sin(alpha)*v;
        pp.pose.position = p;
        geometry_msgs::Quaternion q;
        q.w = cos(alpha/2);
        q.z = sin(alpha/2);
        pp.pose.orientation = q;
        parts.publish(pf.particles);
        truepub.publish(pp);
        pf.update_particles_position(v, w);
        pf.update_particles_weight();
        alpha += w;
        ros::spinOnce();
        rate.sleep();
    }
}
