#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
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
        ros::Subscriber laser_sub;
        ros::Subscriber odomdiff_sub;
        ros::Publisher location_pub;
        double lidar_displ_x;
        double lidar_displ_y;
        double lidar_displ_omega;
        tf::TransformBroadcaster br;
        float ang_noise_factor;
        float lin_noise_factor;

        
    public:
      
        bool hasScan;
        void reInitiateParticles( const std_msgs::Bool::ConstPtr &msg);
        ParticleFilter(geometry_msgs::PoseStamped initialPose,
                       int _nParticles,
                       int _nScans,
                       std::vector<Line> _map, 
                       float ang_noise_factor_,
                       float lin_noise_factor_);
        void update_lastPose();
        tf::TransformListener listener;
        geometry_msgs::PoseStamped lastPose;
        sensor_msgs::LaserScan latest_scan;
        sensor_msgs::PointCloud particles;
        std::vector<float> rayTrace(const geometry_msgs::Point32 &pos,
                                    float angle,
                                    const std::vector<float> &angles);
        void update_particles_position(ros::Time t, float ang_noise_f, float lin_noise_f);
        void update_particles_weight();
        void resample_particles(float noise);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

bool myfunction (int i,int j) { return (i>j); }

ParticleFilter::ParticleFilter(geometry_msgs::PoseStamped initialPose,
                               int _nParticles,
                               int _nScans,
                               std::vector<Line> _map,
                               float ang_noise_factor_,
                               float lin_noise_factor_):nParticles(_nParticles), 
                                                       vel_noise(0,0.1),
                                                       n(),nScans(_nScans),
                                                       ang_noise(0,0.05), 
                                                       particles(), map(_map),
                                                       hasScan(false),
                                                       br(),
                                                       ang_noise_factor(ang_noise_factor_),
                                                       lin_noise_factor(lin_noise_factor_)
{   
    initialPose.header.stamp = ros::Time::now();
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("robot","laser",
                                  ros::Time(0), ros::Duration(2));
        listener.lookupTransform("robot","laser",
                                 ros::Time(0), transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("particle update failed");
    }

    lidar_displ_x = transform.getOrigin().x();
    lidar_displ_y = transform.getOrigin().y();
    lidar_displ_omega = tf::getYaw(transform.getRotation());
    lastPose = initialPose;
    initiateParticles(nParticles);
    laser_sub = n.subscribe("scan", 1, &ParticleFilter::scanCallback, this);
    odomdiff_sub = n.subscribe("robot/odomdiff", 1, &ParticleFilter::reInitiateParticles, this);
}

void ParticleFilter::reInitiateParticles( const std_msgs::Bool::ConstPtr &msg)
{   
    //initiateParticles(nParticles);
}
void ParticleFilter::initiateParticles( int nParticles)
{
    
    tf::StampedTransform transform;
    geometry_msgs::Point initialPoint = lastPose.pose.position;
    double initialAng = atan2(lastPose.pose.orientation.z,
                              lastPose.pose.orientation.w)*2;

    std::vector<geometry_msgs::Point32> ps(nParticles); 
    std::vector<sensor_msgs::ChannelFloat32> ch(nParticles);

    for ( int i = 0; i < nParticles; i++)
    {
        geometry_msgs::Point32 p;
        p.x = initialPoint.x + (float)vel_noise(rng)*2;
        p.y = initialPoint.y + (float)vel_noise(rng)*2;
        p.z = 0;
        ps[i] = p;
        std::vector<float> values(2);
        values[0] = initialAng + ang_noise(rng)*2;
        values[1] = 1;
        ch[i].values = values;
    }

    particles.header.frame_id = "map"; 
    particles.header.stamp = ros::Time();
    particles.points = ps;
    particles.channels = ch;
}

void ParticleFilter::resample_particles(float noise)
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
        newPs[i].x += vel_noise(rng)*noise;
        newPs[i].y += vel_noise(rng)*noise;
        newCh[i] = particles.channels[ind];
        newCh[i].values[0] += vel_noise(rng)*noise;
    }

    particles.points = newPs;
    particles.channels = newCh;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    hasScan = true;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(lastPose.pose.position.x, lastPose.pose.position.y, 0));
    tf::Quaternion q;
    tf::quaternionMsgToTF(lastPose.pose.orientation, q);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, latest_scan.header.stamp , "map", "est_pos"));

    update_particles_position(msg->header.stamp, ang_noise_factor, lin_noise_factor);
    latest_scan = sensor_msgs::LaserScan(*msg);
}

void ParticleFilter::update_particles_position(ros::Time t, float ang_noise_f, float lin_noise_f)
{
   
    tf::StampedTransform transform;
    try{
        listener.waitForTransform("odom",latest_scan.header.stamp,"odom",t ,"map",ros::Duration(1));
        listener.lookupTransform("odom",latest_scan.header.stamp,"odom",t ,"map", transform);
    } catch(tf::TransformException &ex)
    {
        ROS_INFO("particle update failed");
        std::cout << ex.what() << "\n";
    }
    for ( int i = 0; i < nParticles; i++)
    {
        double alpha = particles.channels[i].values[0] 
                     + atan2(transform.getOrigin().y(),transform.getOrigin().x());
        double vt = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
        double wt = tf::getYaw(transform.getRotation());
        double omega_n = particles.channels[i].values[0]
                       + wt*(1 + ang_noise(rng)*ang_noise_f) + ang_noise(rng)*ang_noise_f;

        geometry_msgs::Point32 p = particles.points[i];
        particles.channels[i].values[0] = omega_n;
        particles.points[i].x += cos(alpha)*vt 
                               + vt*(float)vel_noise(rng)*cos(alpha)*lin_noise_f*100;
        particles.points[i].y += sin(alpha)*vt 
                               + vt*(float)vel_noise(rng)*sin(alpha)*lin_noise_f*100;
    }
}

void ParticleFilter::update_lastPose()
{
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

    lastPose.header.stamp = latest_scan.header.stamp;
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
    float var = 0.5;
    double angle_increment = 2*PI/nAngles;

    //find values that are not infinate in the laser.
    for(int ind = 0; ind<nAngles;ind++)
    {
        if (laser_values[ind]<3) //should really check if value not == inf, but who cares?                           
            //  We should not have values larger than 10 either

        {
            inds.push_back(ind);
        }
    }
    int toScan = std::min(nScans, (int)inds.size());
   

    for (int i=0; i<nParticles;i++){
        geometry_msgs::Point32 p = particles.points[i];
        float angle = particles.channels[i].values[0];
        std::vector<float> angles(toScan);
        std::vector<float> dists(toScan);
        std::shuffle(inds.begin(), inds.end(), rng);

        for (int j=0; j<toScan; j++)
        {
            angles[j]=inds[j]*angle_increment;
            dists[j] = laser_values[inds[j]];
        }

        std::vector<float> rays = rayTrace(p, angle, angles);

        float weight = 0;
        std::vector<float> weights(toScan);
        for (int j=0; j<toScan; j++)
        {
            weights[j] = -pow((rays[j] - dists[j]), 2.0)/var;
        }
        std::sort(weights.begin(), weights.end(), myfunction);
        for (int j=0; j<toScan - int(toScan/4); j++)
        {
            weight += weights[j];
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
    ros::Publisher parts = n.advertise<sensor_msgs::PointCloud>("particles", 1);
    ros::Publisher truepose = n.advertise<geometry_msgs::PoseStamped>("/localisation/pose", 1);


    double x_start;
    nh.param<double>("x_start", x_start, 0.22);
    double y_start;
    nh.param<double>("y_start", y_start, 0.22);

    geometry_msgs::PoseStamped pp;
    pp.pose.position.x = x_start;
    pp.pose.position.y = y_start;

    pp.header.frame_id = "map";
    pp.header.stamp = ros::Time::now();


    double omega_start;
    nh.param<double>("omega_start",omega_start, PI/2);
    float alpha =omega_start;
    pp.pose.orientation.w = cos(alpha/2);
    pp.pose.orientation.z = sin(alpha/2);
    ROS_INFO("x_start %f, ystart %f, omega_start %f", x_start, y_start, omega_start);
    
    ParticleFilter pf(pp, 1000,8, map, 0.1, 0.1);   

    ros::Rate rate(100);
    while (ros::ok()){
        if (pf.hasScan){
            pf.update_particles_weight();
            pf.resample_particles(0.05);
        }
        pf.update_lastPose();
        parts.publish(pf.particles);
        truepose.publish(pf.lastPose); 
        ros::spinOnce();
        rate.sleep();
    }
}
