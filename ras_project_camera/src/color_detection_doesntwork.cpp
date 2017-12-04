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
    object_flag_pub = nh_.advertise<std_msgs::Bool>("/camera/object_detected",1);

    cv::namedWindow(OPENCV_WINDOW);

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
    detected = msg->data;
    //std::cerr << "detected" << detected << std::endl;
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
    /*
    cv::resize(rgb_frame, rgb_resized, cv::Size(), 1.0, 1.0);
    std::cerr << "rgb_resized size " << rgb_resized.size() << std::endl;
    cv::Mat samples(rgb_resized.rows * rgb_resized.cols, 3, CV_32F);
    std::cerr << "samples size " << samples.size() << std::endl;
    for( int y = 0; y < rgb_resized.rows; y++ )
      for( int x = 0; x < rgb_resized.cols; x++ )
        for( int z = 0; z < 3; z++)
          samples.at<float>(y + x*rgb_resized.rows, z) = rgb_resized.at<Vec3b>(y,x)[z];

    int claster_num = 3;
    cv::Mat labels;
    int attempts = 5;
    cv::Mat centers; 
    cv::kmeans(samples, claster_num, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 100, 2), attempts, KMEANS_RANDOM_CENTERS, centers);
    
    std::cerr << "claster_num " << centers << std::endl;

    cv::Mat new_image( rgb_resized.size(), rgb_resized.type());
    std::cerr << "new_image " << new_image.size() << std::endl;
  for( int j = 0; j < rgb_resized.rows; j++ )
    for( int i = 0; i < rgb_resized.cols; i++ )
    { 
      int cluster_idx = labels.at<int>(j + i*rgb_resized.rows,0);
      new_image.at<Vec3b>(j,i)[0] = centers.at<float>(cluster_idx, 0);
      new_image.at<Vec3b>(j,i)[1] = centers.at<float>(cluster_idx, 1);
      new_image.at<Vec3b>(j,i)[2] = centers.at<float>(cluster_idx, 2);
    }
  cv::imshow(OPENCV_WINDOW, new_image);  
  cv::waitKey(3);
    
    int maximum = -1; int index = -1; int count = 0;
    int classes[claster_num];
    for (int i=0;i<labels.rows;i++)
    {
      int idx = labels.at<int>(i,0);
      classes[idx]++;
      if (classes[idx] > maximum)
      {
        maximum = classes[idx];
        index = idx;
     }
    }
    cv::Mat rgb_filtered_frame;
    std::cerr << "value of rgb " << centers.at<double>(0,0) << std::endl;
    int r_min = int (centers.at<double>(index,0) - 5);
    int r_max = int (centers.at<double>(index,0) + 5);
    int g_min = int (centers.at<double>(index,0) - 5);
    int g_max = int (centers.at<double>(index,0) + 5);
    int b_min = int (centers.at<double>(index,0) - 5);
    int b_max = int (centers.at<double>(index,0) + 5);

    cv::Scalar min(r_min,g_min,b_min);
    cv::Scalar max(r_max,g_max,b_max);

    cv::inRange(rgb_frame, min, max, rgb_filtered_frame);

    cv::cvtColor(rgb_filtered_frame, hsv_frame, CV_RGB2HSV);
    cv::imshow(OPENCV_WINDOW, rgb_filtered_frame);  
    cv::waitKey(3);*/
    cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);
    lastReading = msg->header.stamp;
    has_image = true;
    
  }

  cv::Mat color_filter(const cv::Mat& hsv_frame, int h_min, int s_min, int v_min,
    int h_max, int s_max, int v_max)
 {
  cv::Mat thresholded;
      std::cerr << "in color filter " <<  std::endl;

  cv::Scalar min(h_min,s_min,v_min);
  cv::Scalar max(h_max,s_max,v_max);

  cv::inRange(hsv_frame, min, max, thresholded);
  // Morphological opening
  cv::erode(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  cv::dilate(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  // Morphological closing 
  cv::dilate(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));
  cv::erode(thresholded, thresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph,morph)));

  /*/cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> heirarchy;
  
  {
  cv::Point2f c;
  float r;

  cv::minEnclosingCircle(contours[i], c, r);
  {
    center.push_back(c);
    radius.push_back(r);
  }
  // Visualise
  size_t counts = center.size();
  cv::Scalar red(255,0,0);
  for( int i = 0; i < counts; i++)
  {
    cv::circle(thresholded_frame, center[i], radius[i], red, 3);
  }
  cv::imshow(OPENCV_WINDOW, thresholded_frame);
  cv::waitKey(3);
  }*/

  return thresholded;
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
    std::cerr << "minTargetRadius, maxTargetRadius " << ic.minTargetRadius << " " << ic.maxTargetRadius << std::endl;
    if (ic.has_image) 
    {
      static int image_count = 0;
      
      //color processing

      /*std::vector<cv::Point2i> green_center = ic.color_filter(ic.hsv_frame, ic.green_h_min,ic.green_s_min,ic.green_v_min,
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

      std::cerr << "-----------------" << std::endl;
      //std::vector<vector<Point2i> > centers;
      
      std::cerr << "green center " << green_center << std::endl;
      centers.push_back(green_center);
      std::cerr << "blue_center " << blue_center << std::endl;
      centers.push_back(blue_center);
      std::cerr << "red_center " << red_center << std::endl;
      centers.push_back(red_center);
      //std::cerr << "yellow_center " << yellow_center << std::endl;
      //centers.push_back(yellow_center);
      std::cerr << "purple_center " << purple_center << std::endl;
      centers.push_back(purple_center);
      std::cerr << "orange_center " << orange_center << std::endl;
      centers.push_back(orange_center);*/
      cv::Mat thresholded_frame1 = ic.color_filter(ic.hsv_frame,ic.green_h_min,ic.green_s_min,ic.green_v_min,ic.green_h_max,ic.green_s_max,ic.green_v_max);
/*      cv::Mat thresholded_frame2 = ic.color_filter(ic.hsv_frame,ic.blue_h_min,ic.blue_s_min,ic.blue_v_min,ic.blue_h_max,ic.blue_s_max,ic.blue_v_max);
      cv::Mat thresholded_frame3 = ic.color_filter(ic.hsv_frame,ic.red_h_min,ic.red_s_min,ic.red_v_min,ic.red_h_max,ic.red_s_max,ic.red_v_max);
      cv::Mat thresholded_frame4 = ic.color_filter(ic.hsv_frame,ic.yellow_h_min,ic.yellow_s_min,ic.yellow_v_min,ic.yellow_h_max,ic.yellow_s_max,ic.yellow_v_max);
      cv::Mat thresholded_frame5 = ic.color_filter(ic.hsv_frame,ic.purple_h_min,ic.purple_s_min,ic.purple_v_min,ic.purple_h_max,ic.purple_s_max,ic.purple_v_max);
      cv::Mat thresholded_frame6 = ic.color_filter(ic.hsv_frame,ic.orange_h_min,ic.orange_s_min,ic.orange_v_min,ic.orange_h_max,ic.orange_s_max,ic.orange_v_max);
    */
      /*thresholded_frame = max(thresholded_frame1,thresholded_frame2);
      thresholded_frame = max(thresholded_frame,thresholded_frame3);
      thresholded_frame = max(thresholded_frame,thresholded_frame4);
      thresholded_frame = max(thresholded_frame,thresholded_frame5);
      thresholded_frame = max(thresholded_frame,thresholded_frame6);
*/
      thresholded_frame = thresholded_frame1;
      cv::vector<cv::vector<cv::Point> >contours;
      cv::vector<cv::Vec4i> heirarchy;
      cv::vector<cv::Point2i> center;
      cv::vector<int> radius;
      cv::findContours(thresholded_frame.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
      size_t count = contours.size();
      for( int i=0; i < count; i++)
      {
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle(contours[i], c, r);
        if ( r >= ic.minTargetRadius && r <= ic.maxTargetRadius)
 	{
          center.push_back(c);
	  radius.push_back(r);
	}
      }
      std_msgs::Bool object_flag;
      geometry_msgs::PointStamped object_coord;
      object_coord.header.stamp = ic.lastReading;
      
      size_t counts = center.size();
      std::cerr << "number of objects " << counts << std::endl;
      //std::cerr << "x and y of centers" << centers << std::endl;
      if (counts!=0)
      { 
        object_flag.data = 1;

        object_coord.point.x = center[0].x;
        object_coord.point.y = center[0].y;
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

      //cv::imshow(OPENCV_WINDOW, thresholded_frame);
      //cv::waitKey(3);  
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
