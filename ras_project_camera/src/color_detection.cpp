#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/objdetect/objdetect.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <stdlib.h>
#include <sstream>


using namespace cv;
//static const std::string OPENCV_WINDOW = "Image window";

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
  //ros::Publisher object_speaker_pub;
  cv::Mat hsv_frame;
  cv::Mat rgb_frame;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Time lastReading;
  bool has_image;
  int green_h_max, green_s_max, green_v_max, green_h_min, green_s_min, green_v_min, morph, nb_of_colors;
  int blue_h_max, blue_s_max, blue_v_max, blue_h_min, blue_s_min, blue_v_min;
  int red_h_max, red_s_max, red_v_max, red_h_min, red_s_min, red_v_min;
  int yellow_h_max, yellow_s_max, yellow_v_max, yellow_h_min, yellow_s_min, yellow_v_min;
  int purple_h_max, purple_s_max, purple_v_max, purple_h_min, purple_s_min, purple_v_min;
  int orange_h_max, orange_s_max, orange_v_max, orange_h_min, orange_s_min, orange_v_min;
  int obstacle_h_max, obstacle_s_max, obstacle_v_max, obstacle_h_min, obstacle_s_min, obstacle_v_min;
  
  int minTargetRadius, maxTargetRadius;
  bool detected;

  ImageConverter(): it_(nh_), has_image(0), hsv_frame(), cv_ptr(), minTargetRadius(50), maxTargetRadius(150)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::ImageCb, this);
    image_pub_ = it_.advertise("/camera/object_detected_image", 1);
    //detection_sub = nh_.subscribe("/camera/detection", 1, &ImageConverter::DetectionCb, this);
    object_coord_pub = nh_.advertise<geometry_msgs::PointStamped>("/camera/object_coord",1);
    object_flag_pub = nh_.advertise<std_msgs::Bool>("/camera/color_detected",1);
   // object_speaker_pub = nh_.advertise<std_msgs::String>("/espeak/string",1); 
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
    detected = true;
    //std::cerr << "detected" << detected << std::endl;
  }

  void BarcodeCb(const std_msgs::String msg)
  {
    barcode_detected = true;
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
    
    cv::Mat rgb_resized;
    cv::GaussianBlur(cv_ptr->image, rgb_frame, cv::Size(9,9),50);
    cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);
    lastReading = msg->header.stamp;
    has_image = true;
  }

  std::vector<cv::Point2i> color_filter(const cv::Mat& hsv_frame, int h_min, int s_min, int v_min,
    int h_max, int s_max, int v_max)
 {
  cv::Mat thresholded;
      //std::cerr << "in color filter " <<  std::endl;

  cv::Scalar min(h_min,s_min,v_min);
  cv::Scalar max(h_max,s_max,v_max);

  cv::inRange(hsv_frame, min, max, thresholded);
  // Morphological opening
  cv::erode(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  cv::dilate(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  // Morphological closing 
  cv::dilate(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  cv::erode(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));

  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> heirarchy;
  cv::vector<cv::Point2i> center;
  cv::vector<int> radius;
  cv::findContours(thresholded.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
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
  // Visualise
  size_t counts = center.size();
  cv::Scalar red(255,0,0);
  for( int i = 0; i < counts; i++)
  {
    cv::circle(thresholded, center[i], radius[i], red, 3);
  }
  //cv::imshow(OPENCV_WINDOW, thresholded);
  //cv::waitKey(3);
  
  return center;
  }
};

int main(int argc, char* argv[]) //int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detection");
  ImageConverter ic;
  //std::cerr << "in main" << std::endl;
  ros::Rate loop_rate(10);
  cv::Mat thresholded_frame;
  while (ros::ok())
  {
    //std::cerr << "minTargetRadius, maxTargetRadius " << ic.minTargetRadius << " " << ic.maxTargetRadius << std::endl;
    if (ic.has_image) 
    {
      static int image_count = 0;
      
      //color processing

      std::vector<cv::Point2i> green_center = ic.color_filter(ic.hsv_frame, ic.green_h_min,ic.green_s_min,ic.green_v_min,
        ic.green_h_max,ic.green_s_max,ic.green_v_max);

      std::vector<cv::Point2i> blue_center = ic.color_filter(ic.hsv_frame, ic.blue_h_min,ic.blue_s_min,ic.blue_v_min,
        ic.blue_h_max,ic.blue_s_max,ic.blue_v_max);

      std::vector<cv::Point2i> red_center = ic.color_filter(ic.hsv_frame, ic.red_h_min,ic.red_s_min,ic.red_v_min,
        ic.red_h_max,ic.red_s_max,ic.red_v_max);

      std::vector<cv::Point2i> yellow_center = ic.color_filter(ic.hsv_frame, ic.yellow_h_min,ic.yellow_s_min,ic.yellow_v_min,
        ic.yellow_h_max,ic.yellow_s_max,ic.yellow_v_max);
      std::vector<cv::Point2i> purple_center = ic.color_filter(ic.hsv_frame, ic.purple_h_min,ic.purple_s_min,ic.purple_v_min,
        ic.purple_h_max,ic.purple_s_max,ic.purple_v_max);

      std::vector<cv::Point2i> orange_center = ic.color_filter(ic.hsv_frame, ic.orange_h_min,ic.orange_s_min,ic.orange_v_min,
        ic.orange_h_max,ic.orange_s_max,ic.orange_v_max);

      std::vector<cv::Point2i> centers;
      centers.insert( centers.end(), green_center.begin(), green_center.end() );
      centers.insert( centers.end(), blue_center.begin(), blue_center.end() );
      centers.insert( centers.end(), red_center.begin(), red_center.end() );
      centers.insert( centers.end(), yellow_center.begin(), yellow_center.end() );
      centers.insert( centers.end(), purple_center.begin(), purple_center.end() );
      centers.insert( centers.end(), orange_center.begin(), orange_center.end() );
      
      std_msgs::Bool object_flag;
      //std_msgs::String speaker_flag;
      geometry_msgs::PointStamped object_coord;
      object_coord.header.stamp = ic.lastReading;
      size_t counts = center.size();
      //std::cerr << "number of objects" << counts << std::endl;

      if (counts!=0)
      { 
        int y_max = -1;
        int idx = -1;
        for (int j = 0; j<counts; ++j)
        {
          cv::circle(thresholded_frame, center[i], radius[i], red, 3);
        }
        object_flag.data = 1;
       // speaker_flag.data = "An object";
        	if (ic.detected)// and !ic.barcode_detected)
        	{
          //Publish object coord
          object_coord.point.x = center[0].x;
          object_coord.point.y = center[0].y;
          ic.object_coord_pub.publish(object_coord);

          //Publish object detected image 
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ic.lastReading;
          out_msg.encoding = sensor_msgs::image_encodings::BGR8;
          out_msg.image = ic.cv_ptr->image;
          ic.image_pub_.publish(out_msg.toImageMsg());
          ic.detected = 0;
          ic.barcode_detected = 0;
          }
        }

        object_flag.data = 1;

        object_coord.point.x = centers[idx].x;
        object_coord.point.y = centers[idx].y;
        ic.object_coord_pub.publish(object_coord);

        //Publish object detected image 

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ic.lastReading;
        //out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC2; 
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = ic.cv_ptr->image;
        ic.image_pub_.publish(out_msg.toImageMsg());

        /*// save images                               
        std::stringstream sstream;                               
        sstream << "object" << image_count << ".jpg" ;                  
        ROS_ASSERT( cv::imwrite( sstream.str(), ic.cv_ptr->image) );      
        image_count++;
        std::cerr << "im " << image_count << std::endl;*/
      }

      else
      {
        object_flag.data = 0;
      }
      
      ic.object_flag_pub.publish(object_flag);
     // ic.object_speaker_pub.publish(speaker_flag);

      //cv::imshow(OPENCV_WINDOW, thresholded_frame);
      //cv::waitKey(3);  
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
