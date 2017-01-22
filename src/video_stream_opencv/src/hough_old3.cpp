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

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image Window";

int cnt = 0;      
int thres = 75;
int minthres = 75;
int min_dis = 100;
double IJ[5][2];
int skip = 0;

class HoughDetecter
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  std_msgs::String msg;

  ros::Publisher pub_c1;
  ros::Publisher pub_c2;
  ros::Publisher pub_c3;
  ros::Publisher pub_c4;
  ros::Publisher pub_c5;

  geometry_msgs::Pose2D c1;
  geometry_msgs::Pose2D c2;
  geometry_msgs::Pose2D c3;
  geometry_msgs::Pose2D c4;
  geometry_msgs::Pose2D c5;

  public:
    HoughDetecter()
      : it_(nh_)
    {
      image_sub_ = it_.subscribe("/webcam/image_raw", 1, &HoughDetecter::imageCb, this);
      image_pub_ = it_.advertise("/camera/image_processed", 1);
      
      pub_c1 = n.advertise<geometry_msgs::Pose2D>("/block1_coord", 1);
      pub_c2 = n.advertise<geometry_msgs::Pose2D>("/block2_coord", 1);
      pub_c3 = n.advertise<geometry_msgs::Pose2D>("/block3_coord", 1);
      pub_c4 = n.advertise<geometry_msgs::Pose2D>("/block4_coord", 1);
      pub_c5 = n.advertise<geometry_msgs::Pose2D>("/block5_coord", 1);
    }

		~HoughDetecter()
		{
			cv::destroyWindow(WINDOW);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& original_image)
		{
		  cv_bridge::CvImagePtr cv_ptr; //Convert ROS image to CvImage
      try
		  {
			   cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8); //Copy image into cv_ptr
			}

			catch (cv_bridge::Exception& e)
			{
				 ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
         return;
			}

      // Read in image from webcam
      cv::Mat image = cv_ptr -> image;// Read webcam image into image
      cv::Mat image2 = image.clone();// Clone image into image2

      // Blur image to avoid false circle detection
      cv::Mat image_bgr;
      cv::medianBlur(image2, image_bgr, 3);//Apply mediunblur to image2

      // Convert the image from BGR to HSV format
      cv::Mat image_hsv;
      cv::cvtColor(image_bgr, image_hsv, cv::COLOR_BGR2HSV);

      // Threshold image to keep only red pixels
      cv::Mat image_low_threshold;
      cv::Mat image_high_threshold;

      cv::inRange(image_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), image_low_threshold);
      cv::inRange(image_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), image_high_threshold);

      // Combine the two thresholded images
      cv::Mat image_red;
      cv::addWeighted(image_low_threshold, 1.0, image_high_threshold, 1.0, 0.0, image_red);

  		// Apply GaussianBlur to reduce noise and avoid false circle detection
			cv::GaussianBlur(image_red, image_red, cv::Size(9, 9), 2, 2);

      // Apply Hough Circle Transform to detect circles
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(image_red, circles, CV_HOUGH_GRADIENT, 1, min_dis, 100, minthres, 0, 0);


      for(size_t i = 0; i < circles.size(); i++)
      {
          // Draw the detected circles
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = circles[i][2];

          // Draw outline
          cv::circle(image, center, radius,cv::Scalar(0, 255, 0), 2);
          //cv::circle(image, center, radius, cv::Scalar(0,255,0), -1, 8, 0);

          // Draw center point
          cv::circle(image, center, 5, cv::Scalar(0,0,255),2);
          //cv::circle(image, center, 5, cv::Scalar(0,0,255),3,8,0);

          skip = 0;

          // Can initialize IJ with impossible values if necessary

          if (sqrt(((IJ[0][0] - circles[i][0]) * (IJ[0][0] - circles[i][0])) + ((IJ[0][1] - circles[i][1]) * (IJ[0][1] - circles[i][1]))) < thres)
             skip = 1;
          
          if (sqrt(((IJ[1][0] - circles[i][0]) * (IJ[1][0] - circles[i][0])) + ((IJ[1][1] - circles[i][1]) * (IJ[1][1] - circles[i][1]))) < thres)
             skip = 1 ;

          if (sqrt(((IJ[2][0] - circles[i][0]) * (IJ[2][0] - circles[i][0])) + ((IJ[2][1] - circles[i][1]) * (IJ[2][1] - circles[i][1]))) < thres)
             skip = 1;

          if (sqrt(((IJ[3][0] - circles[i][0]) * (IJ[3][0] - circles[i][0])) + ((IJ[3][1] - circles[i][1]) * (IJ[3][1] - circles[i][1]))) < thres)
             skip = 1;

          if (sqrt(((IJ[4][0] - circles[i][0]) * (IJ[4][0] - circles[i][0])) + ((IJ[4][1] - circles[i][1]) * (IJ[4][1] - circles[i][1]))) < thres)
             skip = 1;

          if(skip == 0)
          {
          	 IJ[cnt][0] = circles[i][0];
             IJ[cnt][0] = circles[i][0];
             cnt++;
          }


          /*
          for(int h = 0; h < cnt; h++);
          {
             temp = thres + 5;
             temp = IJ[h][0];
             temp = sqrt(((IJ[h][0] - circles[i][0]) * (IJ[j][0] - circles[i][0])) + ((IJ[j][1] - circles[i][1]) * (IJ[j][1] - circles[i][1])));
             if ( temp < thres)
             {
                skip = 1;
                break;
             }
          }

          if(skip == 0)
          {
            IJ[cnt][0] = circles[i][0];
            IJ[cnt][1] = circles[i][1];
            cnt++;
          }
          */

          ROS_INFO("Number of Blocks Found: %d", cnt);
      }

      // Show images
      cv::imshow("Hough Detector Image", image);
      cv::imshow("Red Hue", image_red);

      if(cnt == 5)
      {
        c1.x = IJ[0][0];
        c1.y = IJ[0][1];
        c2.x = IJ[1][0];
        c2.y = IJ[1][1];
        c3.x = IJ[2][0];
        c3.y = IJ[2][1];
        c4.x = IJ[3][0];
        c4.y = IJ[3][1];
        c5.x = IJ[4][0];
        c5.y = IJ[4][1];

        pub_c1.publish(c1);
        pub_c2.publish(c2);
        pub_c3.publish(c3);
        pub_c4.publish(c4);
        pub_c5.publish(c5);

        ROS_INFO("Block Coordinates Published");
        cnt++;
      }

      cv::waitKey(2);
  	}
};

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "Image_Processor");
  HoughDetecter ic;
  ros::spin();
  return 0;
}

