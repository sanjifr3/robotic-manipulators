#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/features2d/features2d.hpp>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include <string>
#include <ros/console.h>

using namespace Eigen;
using namespace std;
using namespace cv;


void 
{
	std_msgs::String msg;
 	ros::NodeHandle nh_;
  	ros::NodeHandle n;
  	ros::Publisher pub_c1;


  	geometry_msgs::Pose2D c1;


    c1.x = 5;
    c1.y = 10;



    pub_c1.publish(c1);
 

    cv::waitKey(5);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Image_Processor");
  HoughDetecter ic;
  ros::spin();
  return 0;
}