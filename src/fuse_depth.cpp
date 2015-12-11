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
#include <std_msgs/Float64MultiArray.h>

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

cv::Mat frameBGR, depthMap, BGRD, circles;
cv_bridge::CvImage _fused;
ros::Publisher fusedPublisher;

void BGRCallback(const sensor_msgs::ImageConstPtr& msg);
void depthMapCallback(const sensor_msgs::ImageConstPtr& msg);
void fuseDepthwithRGB();
void mergeDepthwithColor();

int main(int argc, char** argv){
  ros::init(argc, argv, "fuse_depth");

  ros::NodeHandle nh;
  image_transport::ImageTransport depthMapTransport(nh);
  image_transport::ImageTransport BGRTransport(nh);
  image_transport::ImageTransport fusedTransport(nh);

  image_transport::Subscriber depthMapSubscriber = depthMapTransport.subscribe("/camera/depth_registered/image_raw", 1, depthMapCallback);
  image_transport::Subscriber BGRSubscriber = BGRTransport.subscribe("/camera/rgb/image_rect_color", 1, BGRCallback);
  fusedPublisher = nh.advertise<std_msgs::Float64MultiArray>("/fused_depth", 1000);

  frameBGR = cv::Mat::zeros(480,640, CV_8UC1);
  depthMap = cv::Mat::zeros(480,640, CV_32FC1);
  BGRD = cv::Mat::zeros(480,640, CV_32FC4);

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
  // Update GUI Window
  cv::imshow("BGR Feed", frameBGR);
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
  // std::cout<<"Depth: "<<depthMap.type()<<depthMap.size()<<cv::sum(depthMap)<<std::endl;
  mergeDepthwithColor();
  // Update GUI Window
  cv::imshow("Depth Feed", cv_ptr->image);
  cv::waitKey(3);
}

void fuseDepthwithRGB() {
	cv::Mat normDistance(BGRD.size(), CV_32FC1);
  cv::Vec4f weights = cv::Vec4f(0.1,0.1,0.1,0.7);
	for(int i=0; i<BGRD.rows; i++) {
		for(int j=0; j<BGRD.cols; j++) {
			cv::Vec4f VoI =BGRD.at<cv::Vec4f>(i,j);
      // cv::Mat norms = cv::Mat::zeros(3,3, CV_32FC1);
			// cv::Vec4f diff = BGRD.at<cv::Vec4f>(i+y,j+x) - VoI;
        double norm = cv::norm(VoI.mul(weights));
				normDistance.at<float>(i,j) = (float)norm;
		}
	}
  double min, max;
  cv::minMaxLoc(normDistance, &min, &max);
  //std::cout<<min<<"-"<<max<<std::endl;
  cv::Mat dst;
  cv::normalize(normDistance, dst, 0, 1, cv::NORM_MINMAX);
  cv::Mat distanceChar, edges;
  dst = dst*255;
  dst.convertTo(distanceChar, CV_8UC1);
  cv::imshow("Norm Char", distanceChar);
  cv::blur(distanceChar, edges, cv::Size(7,7));
  cv::Canny(edges, edges, 20, 75);
  imshow("Canny", edges);
  std_msgs::Float64MultiArray mat_msg;
  mat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mat_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  //mat_msg.layout.data_offset = 32;
  mat_msg.layout.dim[0].label = "rows";
  // number of rows
  mat_msg.layout.dim[0].size = normDistance.rows;
  //number of rows * number of columns
  mat_msg.layout.dim[0].stride = normDistance.rows*normDistance.cols;
  mat_msg.layout.dim[1].label = "columns";
  // number of columns
  mat_msg.layout.dim[1].size = normDistance.cols;
  mat_msg.layout.dim[1].stride = normDistance.cols;

  // loop over row indices and column indices
  for (int rows = 0; rows < mat_msg.layout.dim[0].size; ++rows){
      for (int cols = 0; cols < mat_msg.layout.dim[1].size; ++cols){

          // push back each matrix value
          mat_msg.data.push_back((float)edges.at<uchar>(rows,cols));

      }
  }

  // publish message
  fusedPublisher.publish(mat_msg);
}

void mergeDepthwithColor() {
	cv::Mat _BGR, _depthMap;
	std::vector<cv::Mat> channels;

	frameBGR.convertTo(_BGR, CV_32FC3);
	depthMap.convertTo(_depthMap, CV_32FC1);

	cv::split(_BGR, channels);
  // normalize colors
  for(size_t i=0; i<channels.size();i++){
    double min, max;
    cv::minMaxLoc(channels[i], &min, &max);
    channels[i] = channels[i]/max;
  }
	channels.push_back(_depthMap);

	BGRD.create(frameBGR.size(),CV_32FC4);
	cv::merge(channels, BGRD);
  fuseDepthwithRGB();
}
