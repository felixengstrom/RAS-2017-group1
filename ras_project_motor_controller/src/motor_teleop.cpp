#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(0.2),
  a_scale_(0.8)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("motor_teleop/twist", 1000);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;
  ros::Rate loop_rate(100);

  signal(SIGINT,quit);
  teleop_turtle.keyLoop();
}

void TeleopTurtle::keyLoop()
{
  char c;
  static bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        linear_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        linear_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        angular_ = 0.0;
        dirty = true;
        break;
      case ' ':
        ROS_DEBUG("DOWN");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}


/*#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
using namespace std;

#define KEYCODE_R 'd' 
#define KEYCODE_L 'a'
#define KEYCODE_U 'w'
#define KEYCODE_D 's'

int main (int argc, char **argv)
{
  ros::init(argc, argv, "motor_teleop");
  ros::NodeHandle n_;
  ros::Publisher teleop_pub;

  teleop_pub = n_.advertise<geometry_msgs::Twist>("motor_teleop/cmd_vel", 1000);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    char c;
    geometry_msgs::Twist msg;
    cin.get(c);
    printf("%c", c);
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    if (c == KEYCODE_R)
    {
      msg.linear.x = 0;
      msg.angular.z = 3.0;
    }
    else if (c == KEYCODE_L)
    {
      msg.linear.x = 0;
      msg.angular.z = -3.0;
    }
    else if (c == KEYCODE_U)
    {
      msg.linear.x = 1.0;
      msg.angular.z = 0;
    }
    else if (c == KEYCODE_D)
    {
      msg.linear.x = -1.0;
      msg.angular.z = 0;
    }
    teleop_pub.publish(msg);
    loop_rate.sleep();
  }
}*/
