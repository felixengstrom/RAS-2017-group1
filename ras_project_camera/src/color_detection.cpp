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

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher object_coord_pub;
  ros::Publisher object_flag_pub;
  int green_h_max, green_s_max, green_v_max, green_h_min, green_s_min, green_v_min, morph, nb_of_colors;
  int blue_h_max, blue_s_max, blue_v_max, blue_h_min, blue_s_min, blue_v_min;
  //int red_h_max, red_s_max, red_v_max, red_h_min, red_s_min, red_v_min;
  int yellow_h_max, yellow_s_max, yellow_v_max, yellow_h_min, yellow_s_min, yellow_v_min;
  bool green;
  int minTargetRadius, maxTargetRadius;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::ImageCb, this);
    image_pub_ = it_.advertise("/camera/image/thresholded_image", 1);
    object_coord_pub = nh_.advertise<geometry_msgs::Point>("/camera/image/object_coord",100);
    object_flag_pub = nh_.advertise<std_msgs::Bool>("/camera/image/object_detected",1);
    cv::namedWindow(OPENCV_WINDOW);
    ros::NodeHandle nh("~");
    //cv::Mat rgb_frame;
    //cv::Mat hsv_frame;
    //cv::Mat thresholded_frame;
/*
    h_max = 80;
    s_max = 255;
    v_max = 255;
    h_min = 40;
    s_min = 0;
    v_min = 0;*/
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
/*
    nh.getParam("red_h_max",red_h_max);
    nh.getParam("red_s_max",red_s_max);
    nh.getParam("red_v_max",red_v_max);
    nh.getParam("red_h_min",red_h_min);
    nh.getParam("red_s_min",red_s_min);
    nh.getParam("red_v_min",red_v_min);
    */
    nh.getParam("yellow_h_max",yellow_h_max);
    nh.getParam("yellow_s_max",yellow_s_max);
    nh.getParam("yellow_v_max",yellow_v_max);
    nh.getParam("yellow_h_min",yellow_h_min);
    nh.getParam("yellow_s_min",yellow_s_min);
    nh.getParam("yellow_v_min",yellow_v_min);

    nh.getParam("morph",morph);
    nh.getParam("nb_of_colors",nb_of_colors);

    nh.getParam("minTargetRadius",minTargetRadius);
    nh.getParam("maxTargetRadius",maxTargetRadius);

  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void ImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      ros::Rate loop_rate(10);
    
    
      // Update GUI Window
     // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
     // cv::waitKey(3);
 
      //Image processing
      cv::Mat rgb_frame;
      cv::Mat hsv_frame;
      cv::Mat thresholded_frame, thresholded_frame1, thresholded_frame2, thresholded_frame3;

      cv::GaussianBlur(cv_ptr->image, rgb_frame, cv::Size(9,9),50);
      cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);
      //thresholdedHSV(green_h_max,green_h_min, green_s_max, green_s_min, green_v_max, green_v_min);
    
      cv::Scalar   green_min(green_h_min,green_s_min,green_v_min);
      cv::Scalar   green_max(green_h_max,green_s_max,green_v_max);

      cv::Scalar   blue_min(blue_h_min,blue_s_min,blue_v_min);
      cv::Scalar   blue_max(blue_h_max,blue_s_max,blue_v_max);

      //cv::Scalar   red_min(red_h_min,red_s_min,red_v_min);
      //cv::Scalar   red_max(red_h_max,red_s_max,red_v_max);

      cv::Scalar   yellow_min(yellow_h_min,yellow_s_min,yellow_v_min);
      cv::Scalar   yellow_max(yellow_h_max,yellow_s_max,yellow_v_max);
  
      //for (int j = 0, j < nb_of_colors 
      cv::inRange(hsv_frame, green_min, green_max, thresholded_frame1);
      cv::inRange(hsv_frame, blue_min, blue_max, thresholded_frame2);
      cv::inRange(hsv_frame, yellow_min, yellow_max, thresholded_frame3);

      thresholded_frame = max(thresholded_frame1, thresholded_frame2);

      // Morphological opening
      cv::erode(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      cv::dilate(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      // Morphological closing 
      cv::dilate(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
      cv::erode(thresholded_frame, thresholded_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  
      cv::vector<cv::vector<cv::Point> > contours;
      cv::vector<cv::Vec4i> heirarchy;
      cv::vector<cv::Point2i> center;
      cv::vector<int> radius;
      //int minTargetRadius = 50;
      //int maxTargetRadius = 250;
      cv::findContours(thresholded_frame.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

      size_t count = contours.size();
    
      for( int i=0; i < count; i++)
      {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle( contours[i], c, r);

        if ( r >= minTargetRadius && r <= maxTargetRadius)
        {
        	center.push_back(c);
        	radius.push_back(r);
        }
      }
      
      std_msgs::Bool object_flag;
      size_t counts = center.size();
      std::cerr << "number of objects" << counts << std::endl;
      //while(ros::ok())
      //{
      if (counts!=0)
      { 
        cv::Scalar red(255,0,0);
        for( int i = 0; i < counts; i++)
        {
          cv::circle(thresholded_frame, center[i], radius[i], red, 3);
        }
      

      int w = hsv_frame.cols;
      std::cerr << "x and y of centers" << center << std::endl;
      geometry_msgs::Point object_coord;
      object_coord.x = center[0].x;
      object_coord.y = center[0].y;
      object_coord.z = 1.0;
      object_coord_pub.publish (object_coord);
      // publish that object is detected
      object_flag.data = 1;
      object_flag_pub.publish(object_flag);
      }
      else
      {
      //publish that object is not detected
      object_flag.data = 0;
      object_flag_pub.publish(object_flag);
      }
      cv_bridge::CvImage out_msg;
      out_msg.header   = msg->header; // Same timestamp and tf frame as input image
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
      out_msg.image    = thresholded_frame; // Your cv::Mat

      image_pub_.publish(out_msg.toImageMsg());


      cv::imshow(OPENCV_WINDOW, thresholded_frame);
      cv::waitKey(3);
      //loop_rate.sleep();
   //}
/*
  // caluclate real world xyz
  // shift im coordinate to the center
  int w = hsv_frame.cols;
  int h = hsv_frame.rows;
  std::cerr << "w and h " << w << h << std::endl;
  int pixel_x = w/2 - center[0].x;
  int pixel_y = h/2 - center[0].y;

  std::cerr << "x and y of centers" << center << std::endl;  
  std::cerr << "x and y of centers" << pixel_x << pixel_y << std::endl;

  // calculate focus
  double focus_x = w/(2*tan(angle_width/2));
  double focus_y = h/(2*tan(angle_height/2));

  // world coordinates 
  double X = pixel_x*
*/
  }
};

int main(int argc, char* argv[]) //int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detection");
  ImageConverter ic;
  ros::spin();
  return 0;
}
