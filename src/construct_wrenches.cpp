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

void circleCallback(const std_msgs::Float64MultiArray::ConstPtr& mat_msg);
void BGRCallback(const sensor_msgs::ImageConstPtr& msg);

cv::Mat circles, frameBGR;

int main(int argc, char **argv) {
  ros::init(argc, argv, "construct_wrenches");
  ros::NodeHandle nh;
  image_transport::ImageTransport BGRTransport(nh);

  ros::Subscriber sub = nh.subscribe("/center_indices_by_votes", 1000, circleCallback);
  image_transport::Subscriber BGRSubscriber = BGRTransport.subscribe("/camera/rgb/image_rect_color", 1, BGRCallback);
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

void circleCallback(const std_msgs::Float64MultiArray::ConstPtr& mat_msg) {
  circles = cv::Mat::zeros(mat_msg->layout.dim[0].size, mat_msg->layout.dim[1].size, CV_32FC1);
  cv::Mat canvas = frameBGR.clone();
  // msg->layout.dim[0] -> number of rows
  // msg->layout.dim[1] -> number of columns
  for (int i=0;i<mat_msg->layout.dim[0].size;++i){
      for (int j=0;j<mat_msg->layout.dim[1].size;++j){
          circles.at<float>(i,j) = mat_msg->data[mat_msg->layout.dim[1].stride*i + j]; //mat_msg->layout.data_offset +
      }

      cv::Point center = cv::Point((int)circles.at<float>(i,1), (int)circles.at<float>(i,0));

      cv::circle(canvas, center, circles.at<float>(i,2), cv::Scalar(0,0,255), 2);
  }

  cv::imshow("Canvas", canvas);
  cv::waitKey(3);

  std::cout << "matrix recieved" << std::endl;
}
