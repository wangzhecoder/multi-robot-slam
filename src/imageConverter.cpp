/*
 * imageConverter.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: turtlebot
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/core/core.hpp"
//#include <vector>

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
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow("G");
    cv::destroyWindow("R");
  }

  void orbDetect(cv::Mat & img)
  {
	  cv::Ptr<cv::ORB> orb_detector=cv::ORB::create();
	  std::vector<cv::KeyPoint> keyPoints;
	  //cv::Mat descriptors_1;
	  //std::vector<cv::Mat> splitedImg;
	  //cv::split(cv_ptr->image,splitedImg);
	  orb_detector->detect(img,keyPoints,cv::Mat());
	  cv::drawKeypoints(img,keyPoints,img);
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
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      orbDetect(cv_ptr->image);
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//      cv::imshow("B",splitedImg[0]);
//      cv::imshow("G",splitedImg[1]);
//      cv::imshow("R",splitedImg[2]);
    cv::waitKey(5000);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}



