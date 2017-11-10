#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <math.h>


//float pixel_x, pixel_y;
cv_bridge::CvImagePtr cv_ptr;
void object_coord_cb (const geometry_msgs::Point::ConstPtr& object_coord_msg)
  {
    float pixel_x = object_coord_msg->x;
    float pixel_y = object_coord_msg->y;
    std::cerr << " x y" <<pixel_x << pixel_y << std::endl;
  }

void depth_image_cb (const sensor_msgs::ImageConstPtr& msg)
{
	//cv_bridge::CvImagePtr cv_ptr;
    //try
    //{
      cv_ptr = cv_bridge::toCvCopy(msg);
      std::cerr << " cv prt" <<cv_ptr << std::endl;
    //}
    //catch (cv_bridge::Exception& e)
    //{
    //  ROS_ERROR("cv_bridge exception: %s", e.what());
    //  return;
    //}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_coord");
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);
  image_transport::Subscriber depth_image_sub;
  ros::Subscriber pixel_sub;
  ros::Publisher object_coord_publisher;

  depth_image_sub = it_.subscribe("camera/depth/image", 1, &depth_image_cb);
  pixel_sub = n.subscribe("/camera/image/object_coord", 1, &object_coord_cb);



  ros::Rate loop_rate(10);
  while (ros::ok())
  {

  // caluclate real world xyz
  // shift im coordinate to the center

  /*int w = cv_ptr.cols;
  int h = cv_ptr.rows;
  std::cerr << "w and h " << w << h << std::endl;
  int pixel_x = w/2 - center[0].x;
  int pixel_y = h/2 - center[0].y;

  float dist = cv_ptr->image(pixel_x, pixel_y);

  std::cerr << "x and y of centers" << center << std::endl;  
  std::cerr << "x and y of centers" << pixel_x << pixel_y << std::endl;

  // calculate focus
  float focus_x = w/(2*tan(angle_width/2));
  float focus_y = h/(2*tan(angle_height/2));

  // world coordinates 
  float X = pixel_x*dist/focus_x;*/

  ros::spinOnce();
  loop_rate.sleep();

}
  return 0;
}
