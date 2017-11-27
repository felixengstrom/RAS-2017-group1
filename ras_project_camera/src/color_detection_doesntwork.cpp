#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <stdlib.h>
#include <sstream>

static const std::string OPENCV_WINDOW = "Image window";


class ImageConverter
{ 
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber detection_sub;
  ros::Publisher object_coord_pub;
  ros::Publisher object_flag_pub;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat rgb_frame;
  cv::Mat hsv_frame;
  cv::Mat thresholded_frame;
  int green_h_max, green_s_max, green_v_max, green_h_min, green_s_min, green_v_min;
  int blue_h_max, blue_s_max, blue_v_max, blue_h_min, blue_s_min, blue_v_min;
  int red_h_max, red_s_max, red_v_max, red_h_min, red_s_min, red_v_min;
  int yellow_h_max, yellow_s_max, yellow_v_max, yellow_h_min, yellow_s_min, yellow_v_min;
  int purple_h_max, purple_s_max, purple_v_max, purple_h_min, purple_s_min, purple_v_min;
  int orange_h_max, orange_s_max, orange_v_max, orange_h_min, orange_s_min, orange_v_min;
  int obstacle_h_max, obstacle_s_max, obstacle_v_max, obstacle_h_min, obstacle_s_min, obstacle_v_min;
  cv::Scalar green_max, green_min, blue_max, blue_min;
  cv::Scalar red_max, red_min, yellow_max, yellow_min;
  cv::Scalar purple_max, purple_min, orange_max, orange_min;
  cv::Scalar obstacle_max, obstacle_min;
  int morph;
  int minTargetRadius, maxTargetRadius;
  bool detected;

  ImageConverter(): it_(nh_), cv_ptr(), detected(0), thresholded_frame(), minTargetRadius(50), maxTargetRadius(250)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::ImageCb, this);
    image_pub_ = it_.advertise("/camera/thresholded_image", 1);
    detection_sub = nh_.subscribe("/camera/detection", 1, &ImageConverter::DetectionCb, this);
    object_coord_pub = nh_.advertise<geometry_msgs::Point>("/camera/object_coord",1);
    object_flag_pub = nh_.advertise<std_msgs::Bool>("/camera/object_detected",1);

    //cv::namedWindow(OPENCV_WINDOW);

    ros::NodeHandle nh("~");

    nh.getParam("green_h_max",green_h_max);
    nh.getParam("green_s_max",green_s_max);
    nh.getParam("green_v_max",green_v_max);
    nh.getParam("green_h_min",green_h_min);
    nh.getParam("green_s_min",green_s_min);
    nh.getParam("green_v_min",green_v_min);

    nh.getParam("blue_h_max",blue_h_max);
    nh.getParam("blue_s_max",blue_s_max);
    nh.getParam("blue_v_max",blue_v_max);
    nh.getParam("blue_h_min",blue_h_min);
    nh.getParam("blue_s_min",blue_s_min);
    nh.getParam("blue_v_min",blue_v_min);

    nh.getParam("red_h_max",red_h_max);
    nh.getParam("red_s_max",red_s_max);
    nh.getParam("red_v_max",red_v_max);
    nh.getParam("red_h_min",red_h_min);
    nh.getParam("red_s_min",red_s_min);
    nh.getParam("red_v_min",red_v_min);

    nh.getParam("yellow_h_max",yellow_h_max);
    nh.getParam("yellow_s_max",yellow_s_max);
    nh.getParam("yellow_v_max",yellow_v_max);
    nh.getParam("yellow_h_min",yellow_h_min);
    nh.getParam("yellow_s_min",yellow_s_min);
    nh.getParam("yellow_v_min",yellow_v_min);

    nh.getParam("purple_h_max",purple_h_max);
    nh.getParam("purple_s_max",purple_s_max);
    nh.getParam("purple_v_max",purple_v_max);
    nh.getParam("purple_h_min",purple_h_min);
    nh.getParam("purple_s_min",purple_s_min);
    nh.getParam("purple_v_min",purple_v_min);

    nh.getParam("orange_h_max",orange_h_max);
    nh.getParam("orange_s_max",orange_s_max);
    nh.getParam("orange_v_max",orange_v_max);
    nh.getParam("orange_h_min",orange_h_min);
    nh.getParam("orange_s_min",orange_s_min);
    nh.getParam("orange_v_min",orange_v_min);

    nh.getParam("obstacle_h_max",obstacle_h_max);
    nh.getParam("obstacle_s_max",obstacle_s_max);
    nh.getParam("obstacle_v_max",obstacle_v_max);
    nh.getParam("obstacle_h_min",obstacle_h_min);
    nh.getParam("obstacle_s_min",obstacle_s_min);
    nh.getParam("obstacle_v_min",obstacle_v_min);

    nh.getParam("morph",morph);
    
    nh.getParam("minTargetRadius",minTargetRadius);
    nh.getParam("maxTargetRadius",maxTargetRadius);

  }

  void DetectionCb(const std_msgs::Bool::ConstPtr& msg)
  {
    bool detected = msg->data;
    std::cerr << "detected" << detected << std::endl;
  }

  void ImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat thresholded_frame1, thresholded_frame2, 
      thresholded_frame3, thresholded_frame4, thresholded_frame5, thresholded_frame6, thresholded_frame7;

      cv::GaussianBlur(cv_ptr->image, rgb_frame, cv::Size(9,9),50);
      cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);

      cv::Scalar green_min(green_h_min,green_s_min,green_v_min);
      cv::Scalar green_max(green_h_max,green_s_max,green_v_max);

      cv::Scalar blue_min(blue_h_min,blue_s_min,blue_v_min);
      cv::Scalar blue_max(blue_h_max,blue_s_max,blue_v_max);

      cv::Scalar red_min(red_h_min,red_s_min,red_v_min);
      cv::Scalar red_max(red_h_max,red_s_max,red_v_max);

      cv::Scalar yellow_min(yellow_h_min,yellow_s_min,yellow_v_min);
      cv::Scalar yellow_max(yellow_h_max,yellow_s_max,yellow_v_max);

      cv::Scalar purple_min(purple_h_min,purple_s_min,purple_v_min);
      cv::Scalar purple_max(purple_h_max,purple_s_max,purple_v_max);

      cv::Scalar orange_min(orange_h_min,orange_s_min,orange_v_min);
      cv::Scalar orange_max(orange_h_max,orange_s_max,orange_v_max);

      cv::Scalar obstacle_min(obstacle_h_min,obstacle_s_min,obstacle_v_min);
      cv::Scalar obstacle_max(obstacle_h_max,obstacle_s_max,obstacle_v_max);

      //Image processing
      
      cv::inRange(hsv_frame, green_min, green_max, thresholded_frame1);
      cv::inRange(hsv_frame, blue_min, blue_max, thresholded_frame2);
      cv::inRange(hsv_frame, red_min, red_max, thresholded_frame3);
      cv::inRange(hsv_frame, yellow_min, yellow_max, thresholded_frame4);
      cv::inRange(hsv_frame, purple_min, purple_max, thresholded_frame5);
      cv::inRange(hsv_frame, orange_min, orange_max, thresholded_frame6);

      thresholded_frame = max(thresholded_frame1, thresholded_frame2);
      thresholded_frame = max(thresholded_frame3, thresholded_frame);
      thresholded_frame = max(thresholded_frame4, thresholded_frame);
      thresholded_frame = max(thresholded_frame5, thresholded_frame);
      thresholded_frame = max(thresholded_frame6, thresholded_frame);

      // Morphological opening
      cv::erode(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      cv::dilate(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      // Morphological closing 
      cv::dilate(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      cv::erode(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));

  }
};

int main(int argc, char* argv[]) //int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detection");
  ImageConverter ic;
  
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> heirarchy;
  cv::vector<cv::Point2i> center;
  cv::vector<int> radius;
  cv::Point2f c;
  float r;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (ic.detected == 1)
      std::cerr << "detected" << ic.detected << std::endl;
    {
      static int image_count = 0;

      cv::findContours(ic.thresholded_frame.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
      size_t count = contours.size();
      for( int i=0; i < count; i++)
      {
        cv::minEnclosingCircle( contours[i], c, r);

        if ( r >= ic.minTargetRadius && r <= ic.maxTargetRadius)
        {
            center.push_back(c);
            radius.push_back(r);
        }
      }

      std_msgs::Bool object_flag;
      geometry_msgs::Point object_coord;
      size_t counts = center.size();
      std::cerr << "number of objects" << counts << std::endl;

      if (counts!=0)
      { 
        cv::Scalar red(255,0,0);
        for( int i = 0; i < counts; i++)
        {
          cv::circle(ic.thresholded_frame, center[i], radius[i], red, 3);
        }

        std::cerr << "x and y of centers" << center << std::endl;
        
        object_flag.data = 1;

        object_coord.x = center[0].x;
        object_coord.y = center[0].y;
        object_coord.z = 0.0;
  
        // save images                             
        std::stringstream sstream;                               
        sstream << "object" << image_count << ".jpg" ;                  
        ROS_ASSERT( cv::imwrite( sstream.str(), ic.cv_ptr->image) );      
        image_count++;
        std::cerr << "im " << image_count << std::endl;
      }

      else
      {
        //publish that object is not detected
        object_flag.data = 0;

        object_coord.x = 0.0;
        object_coord.y = 0.0;
        object_coord.z = 0.0;
      }
      ic.object_coord_pub.publish (object_coord);
      ic.object_flag_pub.publish(object_flag);

      cv_bridge::CvImage out_msg;
      out_msg.header   = ic.cv_ptr->header; // Same timestamp and tf frame as input image
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
      out_msg.image    = ic.thresholded_frame; // Your cv::Mat
      ic.image_pub_.publish(out_msg.toImageMsg());
      cv::imshow(OPENCV_WINDOW, ic.thresholded_frame);
      cv::waitKey(3);
      loop_rate.sleep();
      ros::spinOnce();
    }
    }
  return 0;
}
