#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <geometry_msgs/Point.h>
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //object_coord_pub = it_.advertise<geometry_msgs::Point>("/camera/object_coord",1)
    cv::namedWindow(OPENCV_WINDOW);
  }
  ros::NodeHandle nh("~");
  //int h_max, s_max, v_max, h_min, s_min, v_min;

  int h_max = 80;
  int s_max = 255;
  int v_max = 255;
  int h_min = 40;
  int s_min = 0;
  int v_min = 0;

  nh.getParam("h_max",h_max);
  nh.getParam("s_max",s_max);
  nh.getParam("v_max",v_max);
  nh.getParam("h_min",h_min);
  nh.getParam("s_min",s_min);
  nh.getParam("v_min",v_min);

  //std::string name = "motorcontrol";



  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());

    //Image processing

    cv::Mat rgb_frame;
    cv::Mat hsv_frame;
    cv::Mat thresholded_frame;

    //float s = 0.5;
	//cv::resize( cv_ptr->image, rgb_frame, cv::Size(), s, s, cv::INTER_NEAREST);
	cv::GaussianBlur(cv_ptr->image, rgb_frame, cv::Size(9,9),50);
	cv::cvtColor(rgb_frame, hsv_frame, CV_RGB2HSV);
    //cv::Scalar   min(30,80,60);
    //cv::Scalar   max(80,150,150);

    cv::Scalar   min(h_min,s_min,v_min);
    cv::Scalar   max(h_max,s_max,v_max);

    cv::inRange(hsv_frame, min, max, thresholded_frame);

    cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> heirarchy;
	cv::vector<cv::Point2i> center;
	cv::vector<int> radius;
	int minTargetRadius = 50;
	
	cv::findContours( thresholded_frame.clone(), contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
 
	size_t count = contours.size();
 
	for( int i=0; i<count; i++)
	{
	    cv::Point2f c;
	    float r;
    	cv::minEnclosingCircle( contours[i], c, r);
 
    	if ( r >= minTargetRadius)
    	{
        	center.push_back(c);
        	radius.push_back(r);
    	}
	}

	size_t counts = center.size();
	cv::Scalar red(255,0,0);
 
	for( int i = 0; i < counts; i++)
	{
    	cv::circle(thresholded_frame, center[i], radius[i], red, 3);
	}

	//geometry_msgs::Point object_coord;
	//S_INFO("%d x center coord %d y center  serial_number);
  //object_coord_pub.publish


    cv::imshow(OPENCV_WINDOW, thresholded_frame);
    cv::waitKey(3);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
