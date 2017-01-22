#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string> 
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Empty.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

#define NUMBER_OF_BLOCKS 9

ros::Publisher featurePublisherRed;
ros::Publisher featurePublisherBlue;
ros::Subscriber imageSubscriber;
ros::Subscriber commandSubscriber;
std::vector<geometry_msgs::Pose2D> poseList(NUMBER_OF_BLOCKS);

bool extract = true;

void extractFeature(cv::Mat src);
void extractRed(cv::Mat src);
void extractBlue(cv::Mat src);

void commandCallback(std_msgs::Empty msg) {
  extract = true;
  cout << "Get next image!" << endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  if (extract) {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(*raw, enc::BGR8);
      cv::Mat src = cv_ptr->image;
      extractFeature(src);
      extract = false;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Error! %s", e.what());
      return;
    }
  }
}



/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */
void extractFeature(cv::Mat hj){
  extractRed(hj);
  extractBlue(hj);
}
void extractRed(cv::Mat hj){
  Mat gray_out;
  Mat canny_out;
  Mat rhj = hj;

  cv::cvtColor(rhj, gray_out, CV_BGR2HSV);
  cv::Mat red_hue_image;
//  cv::inRange(gray_out, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_hue_image);
  //cv::inRange(gray_out, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), upper_red_hue_range);
  cv::Mat lower_red_hue_range;
  cv::Mat upper_red_hue_range;
  cv::inRange(gray_out, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), lower_red_hue_range);
  cv::inRange(gray_out, cv::Scalar(160, 50, 50), cv::Scalar(185, 255, 255), upper_red_hue_range);
  
  cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

  geometry_msgs::Pose2D featurePoseRed;

  for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
    cv::Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
    int radius = round(circles[current_circle][2]);

    featurePoseRed.x = circles[current_circle][0];
    featurePoseRed.y = circles[current_circle][1];
    cv::circle(rhj, center, radius, cv::Scalar(0, 255, 0), 5);
  }
//    ROS_INFO("%i, %i",x[0],x[1]);
  cv::imshow( "Red CAMERA", hj);
  cvWaitKey(2);
  ROS_INFO("Red: %f, %f", featurePoseRed.x, featurePoseRed.y);
  featurePublisherRed.publish(featurePoseRed);
}

void extractBlue(cv::Mat hj){
  Mat gray_out;
  Mat canny_out;
  Mat bhj = hj;
  cv::cvtColor(bhj, gray_out, CV_BGR2HSV);
  cv::Mat red_hue_image;
  //cv::inRange(gray_out, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_hue_image);
  cv::inRange(gray_out, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), red_hue_image);

  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

  geometry_msgs::Pose2D featurePoseRed;

  for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
    cv::Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
    int radius = round(circles[current_circle][2]);

    featurePoseRed.x = circles[current_circle][0];
    featurePoseRed.y = circles[current_circle][1];
    cv::circle(bhj, center, radius, cv::Scalar(0, 255, 0), 5);
  }
//    ROS_INFO("%i, %i",x[0],x[1]);
  cv::imshow( "Blue CAMERA", hj);
  cvWaitKey(2);

  ROS_INFO("Blue: %f, %f", featurePoseRed.x, featurePoseRed.y);
  featurePublisherBlue.publish(featurePoseRed);
}
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simpleCanny");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  featurePublisherRed = nh.advertise<geometry_msgs::Pose2D>("/feature/pose2Dred", 1, true);
  featurePublisherBlue = nh.advertise<geometry_msgs::Pose2D>("/feature/pose2Dblue", 1, true);
  imageSubscriber = nh.subscribe("/webcam/image_raw", 1, imageCallback);  
  commandSubscriber = nh.subscribe("/command/extractFeature", 1, commandCallback);

  //cv::Mat src = cv::imread("polygon.png");
  // cv::Mat src = cv::imread("/home/kktsang/catkin_ws/src/me547_project/r4.png");
  // if (src.empty())
  //   return -1;

  // extractFeature(src);

  while (ros::ok())
  {

    loop_rate.sleep();
    ros::spinOnce();
    cv::waitKey(0);
  }
}