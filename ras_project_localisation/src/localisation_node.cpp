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

class ParticleFilter
{
    private:
        int nParticles;
        int nScans;
        ros::NodeHandle n;
        std::default_random_engine rng;
        std::normal_distribution<double> vel_noise;
        std::normal_distribution<double> ang_noise;
        std::vector<Line> map;
        void initiateParticles( int nParticles);
        sensor_msgs::LaserScan fakeScan(const geometry_msgs::Point32 &p,float angle, int n_rays);
        ros::Subscriber laser_sub;
        ros::Publisher location_pub;
        double lidar_displ_x;
        double lidar_displ_y;
        double lidar_displ_omega;
        
    public:
      
        tf::TransformListener listener;
        geometry_msgs::PoseStamped lastPose;
        sensor_msgs::LaserScan latest_scan;
        bool hasScan;
        sensor_msgs::PointCloud particles;
        std::vector<float> rayTrace(const geometry_msgs::Point32 &pos,
                                    float angle,
                                    const std::vector<float> &angles);
        ParticleFilter(geometry_msgs::PoseStamped initialPose,
                       int _nParticles,
                       int _nScans,
                       std::vector<Line> _map);
        void update_particles_position();
        void update_particles_weight();
        void resample_particles();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

ParticleFilter::ParticleFilter(geometry_msgs::PoseStamped initialPose,
                               int _nParticles,
                               int _nScans,
                               std::vector<Line> _map):nParticles(_nParticles), vel_noise(0,0.1),
                                                       n(),nScans(_nScans),
                                                       ang_noise(0,0.05), particles(), map(_map), hasScan(false)
{   
    initialPose.header.stamp = ros::Time::now();
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("robot","laser",ros::Time(0), ros::Duration(2));
        listener.lookupTransform("robot","laser",ros::Time(0), transform);
    } catch(tf::TransformException &ex)
   {
        ROS_INFO("particle update failed");
    }
    lidar_displ_x = transform.getOrigin().x();
    lidar_displ_y = transform.getOrigin().y();
    lidar_displ_omega = tf::getYaw(transform.getRotation());
    ROS_INFO("lidar displasement in relation to robot: x : %f, y : %f, omega : %f", lidar_displ_x, lidar_displ_y,lidar_displ_omega);
    lastPose = initialPose;
    initiateParticles(nParticles);
   /// location_pub  = n.advertise("location"
    laser_sub = n.subscribe("scan", 1, &ParticleFilter::scanCallback, this);
}

void ParticleFilter::initiateParticles( int nParticles)
{
    
    tf::StampedTransform transform;
    geometry_msgs::Point initialPoint = lastPose.pose.position;
    double initialAng = atan2(lastPose.pose.orientation.w, lastPose.pose.orientation.z)*2;
    std::vector<geometry_msgs::Point32> ps(nParticles); 
    std::vector<sensor_msgs::ChannelFloat32> ch(nParticles);

    for ( int i = 0; i < nParticles; i++)
    {
        geometry_msgs::Point32 p;
        p.x = initialPoint.x + (float)vel_noise(rng);
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
void ParticleFilter::resample_particles()
{
    double wSum = 0;
    std::vector<float> weights(nParticles);
    std::vector<geometry_msgs::Point32> newPs(nParticles); 
    std:: vector<sensor_msgs::ChannelFloat32> newCh(nParticles);

    std::map<double, int> cumulative;
    typedef std::map<double, int>::iterator It;

    for ( int i = 0; i < nParticles; i++)
    {
        wSum +=exp(particles.channels[i].values[1]);
    }
    for ( int i = 0; i < nParticles; i++)
    {
        weights[i] = exp(particles.channels[i].values[1])/wSum;
    }
   

    double sum = 0;
    for ( int i = 0; i < nParticles; i++)
    {
        if (weights[i]>10e-10)
        {
            sum += weights[i];
            cumulative[sum] = i;
        }
    }
    
    for ( int i = 0; i < nParticles; i++)
    {
        double uni = rand()*sum/RAND_MAX;  
        int ind = cumulative.upper_bound(uni)->second;
        //std::cout << sum<< " "<< uni<< " " << ind << "\n";
        newPs[i] = particles.points[ind];
        newPs[i].x += vel_noise(rng)*0.1;
        newPs[i].y += vel_noise(rng)*0.1;
        newCh[i] = particles.channels[ind];
        newCh[i].values[0] += vel_noise(rng)*0.1;
    }

    particles.points = newPs;
    particles.channels = newCh;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    hasScan = true;
    latest_scan = sensor_msgs::LaserScan(*msg);
    if (hasScan){
        update_particles_weight();
        resample_particles();
    }
}

void ParticleFilter::update_particles_position()
{
   
    ros::Time t = ros::Time::now();
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("odom",lastPose.header.stamp,"odom",t ,"map",ros::Duration(1));
        listener.lookupTransform("odom",lastPose.header.stamp,"odom",t ,"map", transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("particle update failed");
        std::cout << ex.what() << "\n";
        lastPose.header.stamp = ros::Time::now();
    }
    for ( int i = 0; i < nParticles; i++)
    {
        double vt = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        double wt = tf::getYaw(transform.getRotation());
        double omega_n = particles.channels[i].values[0]+ wt*(1 + ang_noise(rng)) + ang_noise(rng)*0.4;

        geometry_msgs::Point32 p = particles.points[i];
        particles.channels[i].values[0] = omega_n;
        omega_n = omega_n-(std::round(omega_n/(2*PI))*2*PI); //OPTIMATION COULD BE MADE BY REMOVING THIS
        particles.points[i].x += cos(omega_n)*vt + vt*(float)vel_noise(rng)*cos(omega_n);
        particles.points[i].y += sin(omega_n)*vt + vt*(float)vel_noise(rng)*sin(omega_n);
    }

    lastPose.header.stamp = t;


    double mean_x = 0;
    double mean_y = 0;
    double sum_sin = 0;
    double sum_cos = 0;
    for ( int i = 0; i < nParticles; i++)
    {
        mean_x += particles.points[i].x;
        mean_y += particles.points[i].y;
        sum_sin += sin(particles.channels[i].values[0]);
        sum_cos += cos(particles.channels[i].values[0]);

    }
    mean_x = mean_x/nParticles;
    mean_y = mean_y/nParticles;
    double mean_omega = atan2(sum_sin, sum_cos);

    lastPose.header.stamp = t;
    lastPose.pose.position.x= mean_x;
    lastPose.pose.position.y= mean_y;
    lastPose.pose.orientation.w = cos(mean_omega*0.5);
    lastPose.pose.orientation.z = sin(mean_omega*0.5);
}

void ParticleFilter::update_particles_weight()
{
    std::vector<float> laser_values = latest_scan.ranges;
    int nAngles = laser_values.size();

    std::vector<int> inds;
    float var = 1;
    double angle_increment = 2*PI/nAngles;

    //find values that are not infinate in the laser.
    for(int ind = 0; ind<nAngles;ind++)
    {
        if (laser_values[ind]<10) //should really check if value not == inf, but who cares?                           
            //  We should not have values larger than 10 either

        {
            inds.push_back(ind);
        }
    }
    nScans = inds.size();
   

    for (int i=0; i<nParticles;i++){
        geometry_msgs::Point32 p = particles.points[i];
        float angle = particles.channels[i].values[0];
        std::vector<float> angles(nScans);
        std::vector<float> dists(nScans);
        std::shuffle(inds.begin(), inds.end(), rng);

        for (int j=0; j<nScans; j++)
        {
            angles[j]=inds[j]*angle_increment;
            dists[j] = laser_values[inds[j]];
        }

        std::vector<float> rays = rayTrace(p, angle, angles);

        float weight = 0;
        for (int j=0; j<nScans; j++)
        {
            weight += -pow((rays[j] - dists[j])/var, 2.0);
        }

        particles.channels[i].values[1] = weight;
    }
}

std::vector<float> ParticleFilter::rayTrace(const geometry_msgs::Point32 &pos, float angle, const std::vector<float> &angles)
{   
    int n_angles= angles.size();
    std::vector<float> dists(n_angles);
    for(int a = 0; a<n_angles; a++)
    {
        int n_walls = map.size();
        float angle_n = angle + angles[a] + lidar_displ_omega + PI;
        float x_disp = lidar_displ_x*cos(angle) - lidar_displ_y*sin(angle);
        float y_disp = lidar_displ_y*cos(angle) + lidar_displ_x*sin(angle);
        float ca = cos(angle_n);
        float sa = sin(angle_n);
        dists[a] = 10;
        for(int w = 0; w<n_walls; w++)
        {
            float x1 = map[w].x1 - pos.x - x_disp,
                  x2 = map[w].x2 - pos.x - x_disp,
                  y1 = map[w].y1 - pos.y - y_disp,
                  y2 = map[w].y2 - pos.y - y_disp;

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

    

    ros::NodeHandle n;
    ros::Publisher parts = n.advertise<sensor_msgs::PointCloud>("particles", 100);
    ros::Publisher truepose = n.advertise<geometry_msgs::PoseStamped>("robot/pose", 10);

    geometry_msgs::PoseStamped pp;
    std_msgs::Header h;
    h.frame_id = "map";
    h.stamp = ros::Time::now();
    pp.header = h;
    geometry_msgs::Point p;
    p.x = 0.22;
    p.y = 0.22;
    pp.pose.position = p;
    geometry_msgs::Quaternion q;
    float alpha = PI/2;
    q.w = cos(alpha/2);
    q.z = sin(alpha/2);
    pp.pose.orientation = q;
    
    ParticleFilter pf(pp, 1000,40, map);   

    ros::Rate rate(100);
    while (ros::ok()){
        parts.publish(pf.particles);
        //try{
        pf.update_particles_position();
        //} catch (std::Exception &e){
            //ROS_INFO(e.what());
        
        //}
        truepose.publish(pf.lastPose); 
        ros::spinOnce();
        rate.sleep();
        tf::StampedTransform transform;
    }
}
