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
#include <math.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

cv::Mat frameBGR, depthMap, BGRD;
cv_bridge::CvImage _fused;

void BGRCallback(const sensor_msgs::ImageConstPtr& msg);
void depthMapCallback(const sensor_msgs::ImageConstPtr& msg);
void fuse();
void fuseDepthwithColor();

int main(int argc, char** argv){
  ros::init(argc, argv, "fuse_depth");
	ros::NodeHandle nh;
  image_transport::ImageTransport depthMapTransport(nh);
  image_transport::ImageTransport BGRTransport(nh);
  image_transport::ImageTransport fusedTransport(nh);

  image_transport::Subscriber depthMapSubscriber = depthMapTransport.subscribe("/camera/depth/image", 1, depthMapCallback);
  image_transport::Subscriber BGRSubscriber = BGRTransport.subscribe("/camera/rgb/image_rect_color", 1, BGRCallback);
  image_transport::Publisher fusedPublisher = fusedTransport.advertise("/fused_depth", 1);

  ros::spin();

	return 0;
}

void BGRCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    frameBGR = cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  fuseDepthwithColor();
  // Update GUI Window
  cv::imshow("OPENCV_WINDOW", frameBGR);
  cv::waitKey(3);
}

void depthMapCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    depthMap = cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  for(size_t i = 0; i<depthMap.rows;i++)
    for(size_t j = 0; j<depthMap.cols;j++) {
      if(isnan(depthMap.at<float>(i,j)))
        depthMap.at<float>(i,j) = 0;
    }
  std::cout<<"Depth: "<<depthMap.type()<<depthMap.size()<<cv::sum(depthMap)<<std::endl;
  fuseDepthwithColor();
  // Update GUI Window
  cv::imshow("OPENCV_WINDOW 2", cv_ptr->image);
  cv::waitKey(3);
}

void fuse() {
	cv::Mat normDistance(BGRD.size(), CV_32FC1);
  cv::Vec4f weights = cv::Vec4f(0.1,0.1,0.1,0.7);
	for(int i=1; i<BGRD.rows-1; i++) {
		for(int j=1; j<BGRD.cols-1; j++) {
			cv::Vec4f VoI =BGRD.at<cv::Vec4f>(i,j);
			for(int y=-1;y<+2;y++)
				for(int x=-1;x<2;x++) {
					cv::Vec4f diff = BGRD.at<cv::Vec4f>(i+y,j+x) - VoI;
					double norm = cv::norm(BGRD.at<cv::Vec4f>(i+y,j+x).mul(weights));
					normDistance.at<float>(i+y,j+x) = (float)norm;
				}
		}
	}

  cv::Mat dst;
  cv::normalize(normDistance, dst, 0, 1, cv::NORM_MINMAX);
	cv::imshow("l2 norm", dst);
}

void fuseDepthwithColor() {
	cv::Mat _BGR, _depthMap;
	std::vector<cv::Mat> channels;

	frameBGR.convertTo(_BGR, CV_32FC3);
	depthMap.convertTo(_depthMap, CV_32FC1);

	cv::split(_BGR, channels);
	channels.push_back(_depthMap);

	BGRD.create(frameBGR.size(),CV_32FC4);
	cv::merge(channels, BGRD);
  fuse();
}
