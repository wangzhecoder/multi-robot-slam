/*
 * imageConverter.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: turtlebot
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/openfabmap.hpp>
#include <vector>
#include <boost/thread.hpp>
#include <multi_robot_slam/Scenenode.h>

//static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber mark_sub;
  ros::Publisher scene_node_pub;
  bool detSwitch;
  int sceneFrames;
  std::vector<cv::Mat> voc_src;
  cv::Mat vocab;

  typedef struct
  {
	  double x,y;
//	  double angle;
	  std::vector<cv::KeyPoint> keyPoints;
  }sceneNode;
  //std::vector<sceneNode> scene_Node;
  sceneNode s_node;

public:
  ImageConverter();
  ~ImageConverter();
  void orbDetect(cv_bridge::CvImagePtr &cv_ptr,sceneNode node);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void markArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
};

ImageConverter::ImageConverter()
    : it_(nh_),
	  sceneFrames(0)
{
    // Subscrive to input video feed and publish output video feed
	detSwitch = false;
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    mark_sub = nh_.subscribe("visualization_marker_array",1,&ImageConverter::markArrayCallback,this);
    scene_node_pub = nh_.advertise<multi_robot_slam::Scenenode>("/image_converter/scene_node",1);
    //cv::namedWindow(OPENCV_WINDOW);
    vocab = cv::imread("vocab.yml");
}

ImageConverter::~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

void ImageConverter::orbDetect(cv_bridge::CvImagePtr &cv_ptr,sceneNode node)//cv::Mat & img
{

//	cv::Ptr<cv::ORB> orb_detector=cv::ORB::create("orb_detector");
//	cv::ORB orb_detector=cv::ORB();
	cv::Ptr<cv::FeatureDetector> orb_detector =
				cv::FeatureDetector::create("ORB");
	ROS_INFO("orb created");
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptors;
	multi_robot_slam::Scenenode msg;
	sensor_msgs::Image img;
	cv_bridge::CvImage des_ptr;//(cv_ptr->header,sensor_msgs::image_encodings::MONO8,descriptors);
	cv_bridge::CvImagePtr des_ptrr;
	des_ptr.header=cv_ptr->header;
	des_ptr.encoding=sensor_msgs::image_encodings::TYPE_32FC1;
	//std::vector<cv::Mat> splitedImg;
	//cv::split(cv_ptr->image,splitedImg);
	ROS_INFO("before orb created");
//	orb_detector->detect(cv_ptr->image,keyPoints,cv::Mat());
//	cv::imshow("orimg",cv_ptr->image);
//	cv::Mat img32(cv_ptr->image.rows,cv_ptr->image.cols,CV_32FC3);
//	cv::Mat img32;
//	cv::convertScaleAbs(cv_ptr->image, img32, 1.0/255, 0);
//	cv_ptr->image.convertTo(img32, CV_32FC3);
//	cv::imshow("convertimg",img32);
//	orb_detector.operator ()(cv_ptr->image, cv::Mat(), keyPoints, descriptors, 0);
	orb_detector->detect(cv_ptr->image, keyPoints, cv::Mat());
	ROS_INFO("orb_detector detected");
	cv::Ptr<cv::DescriptorExtractor> descriptor_extractor =
		        cv::DescriptorExtractor::create("ORB");
	descriptor_extractor->compute(cv_ptr->image,keyPoints,descriptors);
//	orb_detector.detect(cv_ptr->image, keyPoints, cv::Mat());
//	ROS_INFO("vocab descriptor type:[%d]",vocab.type());

	cv::Mat data;
	descriptors.convertTo(data, CV_32F);
	voc_src.push_back(data);
	if(sceneFrames%1==0&&sceneFrames!=0)
	{
//		cv::BOWKMeansTrainer voc_trainer(100);//100
	//	voc_trainer.add(data);
	//	ROS_INFO("voc added");
//		cv::Mat vocab = voc_trainer.cluster(data);

		cv::Mat vocab_uchar;
		float min=1.0,max=1.0;
		for(int m = 0; m < vocab.rows; m++)
		{
			for(int l = 0; l < vocab.cols; l++)
			{
				if(vocab.at<float>(m,l)<min)
					min=vocab.at<float>(m,l);
				if(vocab.at<float>(m,l)>max)
					max=vocab.at<float>(m,l);
			}
		}
		if(min!=max)
			vocab.convertTo(vocab_uchar, CV_8U,255.0/(max-min),-255.0*min/(max-min));

	//	for(int m = 0; m < vocab_uchar.rows; m++)
	//	for(int l = 0; l < vocab_uchar.cols; l++)
	//	  {
	//		  ROS_INFO("img data:[%d]",vocab_uchar.at<uchar>(m,l));
	//	  }
//		cv::imshow("vocab_uchar", vocab_uchar);
		ROS_INFO("voc clustered");
	//	cv::Ptr<cv::FeatureDetector> detector =
	//			cv::FeatureDetector::create("ORB");
		cv::Ptr<cv::DescriptorExtractor> extractor =
				cv::DescriptorExtractor::create("ORB");

		ROS_INFO("ORB created");
		cv::Ptr<cv::DescriptorMatcher> matcher =
			cv::DescriptorMatcher::create("BruteForce-Hamming");
		ROS_INFO("matcher created");
		cv::Mat kepointImg;
		cv::drawKeypoints(cv_ptr->image,keyPoints,kepointImg);
		cv::imshow("keypoints", kepointImg);
	//	cv::waitKey();
		cv::BOWImgDescriptorExtractor bide(extractor, matcher);
		bide.setVocabulary(vocab_uchar);
		ROS_INFO("bide vocab setted");
	//	std::vector<cv::KeyPoint> kpts;
	//	detector->detect(cv_ptr->image, kpts);
	//	cv::Mat bow;
		bide.compute(cv_ptr->image, keyPoints, des_ptr.image);
//		for(int l = 0; l < des_ptr.image.cols; l++)
//		  {
//			  ROS_INFO("img data:[%f]",des_ptr.image.at<float>(l));
//		  }


		ROS_INFO("bide computed:[%d,%d}",des_ptr.image.type(),des_ptr.image.channels());
//		cv::drawKeypoints(cv_ptr->image,keyPoints,cv_ptr->image);
//		cv::imshow("keypoints", cv_ptr->image);
	//	bow.convertTo(des_ptr.image, CV_8U);
	//	orb_detector->compute(cv_ptr->image,keyPoints,descriptors);
	//	orb_detector->compute(cv_ptr->image,keyPoints,des_ptr.image);
	//	orb_detector.compute(cv_ptr->image, keyPoints, des_ptr.image);
	//	descriptors.depth();
	//	ROS_INFO("computed descriptors:[%d,%d]",descriptors.depth(),descriptors.channels());

	//	des_ptr->header=cv_ptr->header;
	//	des_ptr->encoding=sensor_msgs::image_encodings::MONO8;
	//	des_ptr->CvImage(cv_ptr->header,sensor_msgs::image_encodings::MONO8,descriptors);
	//	ROS_INFO("cv_bridge::CvImagePtr des_ptr;");
	//	des_ptr->image=descriptors;
	//	des_ptr = cv_bridge::
	//	ROS_INFO("des_ptr->image=descriptors;");
		des_ptr.toImageMsg(img);
		ROS_INFO("des_ptr->image SIZE:[%d,%d]",des_ptr.image.rows,des_ptr.image.cols);
		msg.image=img;
		msg.px=node.x;
		msg.py=node.y;
		scene_node_pub.publish(msg);
		ROS_INFO("published");

		voc_src.clear();

	}

//	sceneFrames++;
//	if(sceneFrames%2==0&&sceneFrames!=0)
//	{
//		std::string window_name = "SceneFrames:"+(char)(sceneFrames);
//		cv::drawKeypoints(cv_ptr->image,keyPoints,cv_ptr->image);
//		cv::imshow(window_name, cv_ptr->image);
//	}

	sceneFrames++;
//	try
//	  {
//		 des_ptrr = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::MONO8);
//	  }
//	  catch (cv_bridge::Exception& e)
//	  {
//		 ROS_ERROR("cv_bridge exception: %s", e.what());
//		 return;
//	  }
//
//	cv::imshow("descriptor", des_ptrr->image);
	//s_node.keyPoints = keyPoints;
//	cv::drawKeypoints(cv_ptr->image,keyPoints,cv_ptr->image);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
//	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//	  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	if(detSwitch)
	{
		orbDetect(cv_ptr,s_node);
		//scene_Node.push_back(s_node);
		ROS_INFO("Add SceneNode!Position[%lf,%lf]",s_node.x,s_node.y);
		detSwitch= false;

	}
	// Update GUI Window
//	cv::imshow("des", cv_ptr->image);
	cv::waitKey(33);

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}
void ImageConverter::markArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	//ROS_INFO("Marker:[%lf,%lf]",msg->markers.data()->pose.position.x,msg->markers.data()->pose.position.y);
	//ROS_INFO("Marker:[%lf,%lf]",msg->markers.pop_back());
	//ROS_INFO("SIZE:[%d]",msg->markers[msg->markers.size()-1].pose.position.x);
	//ROS_INFO("Marker:[%lf,%lf,%lf]",msg->markers[0].pose.position.x,msg->markers[0].pose.position.y,msg->markers[0].pose.position.z);
	//ROS_INFO("Marker:[%lf,%lf,%lf]",msg->markers.back().pose.position.x,msg->markers.back().pose.position.y,msg->markers.back().pose.position.z);
	ROS_INFO("Marker:[%lf,%lf]",msg->markers[msg->markers.size()-2].pose.position.x,msg->markers[msg->markers.size()-2].pose.position.y);
	//ROS_INFO("Marker:[%lf,%lf]",msg->MarkerArray_::markers.end()->pose.position.x,msg->MarkerArray_::markers.end()->pose.position.y);

	if(msg->markers.size()>=2)
	{
		s_node.x=msg->markers[msg->markers.size()-2].pose.position.x;
		s_node.y=msg->markers[msg->markers.size()-2].pose.position.y;

	}
	else
	{
		s_node.x=msg->markers.back().pose.position.x;
		s_node.y=msg->markers.back().pose.position.y;
//		s_node.angle=msg->markers.back().pose.orientation;
	}
	detSwitch = true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}



