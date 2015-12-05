//============================================================================
// Name        : MBZIRC-Challenge-2.cpp
// Author      : Murat Ambarkutuk
// Version     : 0.1b
// Copyright   : TODO
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <time.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


int main(int argc, char** argv){
  ros::init(argc, argv, "frame_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport depthImageTransport(nh);
  image_transport::ImageTransport depthMapTransport(nh);
  image_transport::ImageTransport BGRTransport(nh);
  image_transport::ImageTransport PointCloudTransport(nh);

  image_transport::Publisher depthImagePublisher = depthImageTransport.advertise("/depthImage", 1);
  image_transport::Publisher depthMapPublisher = depthMapTransport.advertise("/depthMap", 1);
  image_transport::Publisher BGRImagePublisher = BGRTransport.advertise("/BGR", 1);
  image_transport::Publisher PointCloudPublisher = PointCloudTransport.advertise("/pointCloud", 1);


  cv::Mat frameBGR, depthMap, pointCloud, depthImage;
  cv::VideoCapture capture;
	capture.open(CV_CAP_OPENNI_ASUS);

  cv_bridge::CvImage _frameBGR, _depthMap, _pointCloud, _depthImage;

  _frameBGR.encoding = "bgr8";
  _frameBGR.header.stamp = ros::Time::now();
  _frameBGR.header.frame_id = "/RGBD";

  _depthImage.encoding = "mono8";
  _depthImage.header.stamp = ros::Time::now();
  _depthImage.header.frame_id = "/RGBD";

  _depthMap.encoding = "mono16";
  _depthMap.header.stamp = ros::Time::now();
  _depthMap .header.frame_id = "/RGBD";

  _pointCloud.header.stamp = ros::Time::now();
  _pointCloud .header.frame_id = "/RGBD";


  if(!capture.isOpened()) {
		std::cerr<<"Exiting the software. Cannot communicate with capturing device."<<std::endl;
		return -1;
	}

  cv::namedWindow("BGR Feed and Results",1);
	cv::namedWindow("Depth Feed",1);

	while(true) {
		capture.grab();

		capture.retrieve(frameBGR, CV_CAP_OPENNI_BGR_IMAGE);
		capture.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
		capture.retrieve(pointCloud, CV_CAP_OPENNI_POINT_CLOUD_MAP);

		cv::normalize(depthMap, depthImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    _frameBGR.image = frameBGR.clone();
    _depthImage.image = depthImage.clone();
    _depthMap.image = depthMap.clone();
    _pointCloud.image = pointCloud.clone();

    depthImage.publish(rosImage.toImageMsg());

    cv::imshow("Depth Feed", depthImage);
    cv::imshow("BGR Feed and Results", frameBGR);

    if(cv::waitKey(30)>= 0)
			break;

    ros::spinOnce();
	}
	return 0;
}
