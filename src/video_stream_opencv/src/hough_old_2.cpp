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
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image Window";

float Px = 0.0081;
float Py = 0.0081;
float w = 990+4.3-40;
float fi = 4.3;
float xi,yi;
float I,J;

Eigen::MatrixXf T_ac0(4,4);
Eigen::MatrixXf T_ac1(4,4);
Eigen::MatrixXf T_ac2(4,4);
Eigen::MatrixXf T_as(4,4);
Eigen::MatrixXf T_cs(4,4);
Eigen::MatrixXf p_s(4,1);
Eigen::MatrixXf p_a(4,1);

class HoughDetect
{
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pose_pub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	std_msgs::String msg;
	
	//SimpleBlobDetector blob;
	vector<KeyPoint> keypoints;
  	geometry_msgs::Pose position_msg;

	public:
		HoughDetect()
			: it_(nh_)
		{
     		image_sub_ = it_.subscribe("/camera/image", 1, &HoughDetect::imageCb, this);
     		//image_sub_ = it_.subscribe("/webcam/image_raw", 1, &HoughDetect::imageCb, this);
     		image_pub_= it_.advertise("/camera/image_processed",1);

     		pose_pub=n.advertise<geometry_msgs::Pose>("/position", 20);
		}

		~HoughDetect()
		{
			cv::destroyWindow(WINDOW);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& original_image)
		{
			cv_bridge::CvImagePtr cv_ptr;
    		try
			{
				cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
			}

			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        		return;
			}

			// Load image
			//cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);


		/*
			// Simple Canny Detector Lines
			Mat out1;
			Mat gray_out;
			Mat canny_out;
			Mat gray_out;
			Mat img1;

			cv::cvtColor(cv_ptr->image, gray_out, CV_BGR2GRAY);
      		cv::GaussianBlur(gray_out, gray_out, Size(3, 3), 0, 0);    // Replaces cv::boxFilter(gray_out, gray_out, -1, cv::Size(3,3)); or cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9,9);
      		cv::Canny(gray_out, canny_out, 50, 125, 3);
     		cv::cvtColor(canny_out, gray_out1, CV_GRAY2BGR);
      		cv::imshow( "CAMERA FEED", cv_ptr->image);
      		cv::imshow( "GRAY CAMERA", gray_out);
      		cv::imshow( "CANNY CAMERA", canny_out);
      		cv::imshow( "CANNY EDGE DETECTION",gray_out1);
      		cvWaitKey(2);
      	*/

      	//
      		// Hough Detector

 		 	// Load input image
  			// std::string path_image{argv[1]};

  			cv::Mat bgr_image = cv_ptr->image;
  			cv::Mat orig_image = bgr_image.clone();
  			cv::medianBlur(bgr_image, bgr_image, 3);

  			// Convert image to HSV
  			cv::Mat hsv_image;
  			cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

  			// Threshold the new image to keep only the red pixels
			cv::Mat lower_red_hue_range;
			cv::Mat upper_red_hue_range;
			cv::inRange(hsv_image, cv::Scalar(0, 100, 90), cv::Scalar(10, 255, 255), lower_red_hue_range);
			cv::inRange(hsv_image, cv::Scalar(160, 100, 90), cv::Scalar(179, 255, 255), upper_red_hue_range);

			// Combine the above two images
			cv::Mat red_hue_image;
			cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
			cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

  			// Use the Hough transform to detect circles in the combined threshold image
  			std::vector<cv::Vec3f> circles;
  			cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, 1, 100, 20, 0, 0);

                        //ROS_INFO("Number of detected circles: %d",circles.size() );
          int count = 0;
      		for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) 
      		{
            count = count +1;
        		cv::Point center(circles[current_circle][0], circles[current_circle][1]);
      			/*
      			if(current_circle == 0)
      			{
     				I = (center.x);
     				J = (center.y);	
    			}
    			*/
    			int radius = circles[current_circle][2];

    			cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 2);
  			}

  			xi = Px*I;
  			yi = Py*J;	
        ROS_INFO("Center Point [%f], [%f]", xi, yi);
        ROS_INFO("Number of detected circles: %d", count);

			p_s(0,0) = xi*(fi-w)/fi;
			p_s(1,0) = yi*(fi-w)/fi;
			p_s(2,0) = w;
  			p_s(3,0) = 1;
  			p_a = T_as*p_s;

  			position_msg.position.x = p_a(0,0);
  			position_msg.position.y = p_a(1,0);

		  	//ROS_INFO("Center Point [%f], [%f]", p_a(0,0)/10, p_a(1,0)/10);

  			pose_pub.publish(position_msg);
  			
  			// Show images
  			cv::imshow("Circles ", orig_image);
  			cv::imshow("processed ", red_hue_image);

  			cv::waitKey(50);
  		}
};

int main(int argc, char** argv)
{	
	ros::init(argc,argv,"Image_Processor");

	T_ac0 << 1, 0, 0, 515,
             0, 1, 0,  90,
             0, 0, 1, 990,
             0, 0, 0,   1;

    T_ac1 << -1, 0,  0, 0,
              0, 1,  0, 0,
              0, 0, -1, 0,
   	          0, 0,  0, 1;

   	T_ac2 << 0, -1, 0, 0,
    	     1,  0, 0, 0,
    	     0,  0, 1, 0,
    	     0,  0, 0, 1;

    T_cs << 1, 0, 0,    0,
            0, 1, 0,    0,
            0, 0, 1, -4.3,
            0, 0, 0,    1;

    T_as = T_ac0*T_ac1*T_ac2*T_cs;

    HoughDetect ic;

    ros::spin();

    return 0;
}

/*

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	
	balloonX = msg->position.x;

	balloonY = msg->position.y;
	ROS_INFO("POSE CALLBACK [%f], [%f]", balloonX, balloonY);
}


int main(int argc, char **argv)
{
	ros::Subscriber Pose_sub = armController.nh.subscribe("/position",1,poseCallback);
}

*/
