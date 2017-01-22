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
     		image_sub_ = it_.subscribe("/webcam/image_raw", 1, &HoughDetect::imageCb, this);
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
      		// Hough Detector from Online 
            //http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html




          //Load an image
          //src = imread(argv[1],1);
          //if (!src.data)
          //  { return -1; }

          cv::Mat src_gray;
          cv::Mat src = cv_ptr -> image;
          cv::Mat orig_image = src.clone();
        /*


          //Convert to grayscale
          cvtColor(src,src_gray,CV_BGR2GRAY);

          //Apply Gaussian blur to reduce noise and avoid false circle detection
          GaussianBlur (src_gray, src_gray, Size(9,9),2,2);

          //Apply Hough Circle Transform
          vector <Vec3f> circles;
          HoughCircles (src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/10, 85, 40, 0, 0);
            //input image (grayscale) - src_gray
            //Vector that stores three values (x,y,r) - circles
            //Detection Method - CV_HOUGH_GRADIENT
            //Inverse ratio of resolution (dp) - 1
            //Minimum distance between detected centres (min_dist) - src_gray.rows/8
            //Upper Threshold for internal Canny edge detection (param_1) - 200;
            //Threshold for centre detection (param_2) - 100*
            //Minimum radius to be detected (min_radius) - 0 (by default);
            //Maximum radius to be detected (max_radius) - 0 (by default);

          // Draw the detected circles
          for( size_t i = 0; i < circles.size(); i++ )
          {
              Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
              int radius = cvRound(circles[i][2]);
              // circle center
              circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
          }

          //Show the results
          namedWindow("Hough Circle Transform", CV_WINDOW_AUTOSIZE);
          imshow("Hough Circle Transform", src);

      */





         //V3

              // Load input image
        // std::string path_image{argv[1]};
  			cv::Mat bgr_image = cv_ptr->image;
  			//cv::Mat orig_image = bgr_image.clone();
  			cv::medianBlur(bgr_image, bgr_image, 3);

  			// Convert image to HSV
  			cv::Mat hsv_image;
  			cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

  			// Threshold the new image to keep only the red pixels
			cv::Mat lower_red_hue_range;
			cv::Mat upper_red_hue_range;
			cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
			cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

			// Combine the above two images
			cv::Mat red_hue_image;
			cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
			cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

  			// Use the Hough transform to detect circles in the combined threshold image
  			std::vector<cv::Vec3f> circles;
  			cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 200, 100, 0, 0);

      		for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) 
      		{
        		cv::Point center(circles[current_circle][0], circles[current_circle][1]);
      			if(current_circle == 0)
      			{
     				I = (center.x);
     				J = (center.y);	
    			}
    			int radius = circles[current_circle][2];

    			cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
        }

  			xi = Px*I;
  			yi = Py*J;	

			p_s(0,0) = xi*(fi-w)/fi;
			p_s(1,0) = yi*(fi-w)/fi;
			p_s(2,0) = w;
  			p_s(3,0) = 1;
  			p_a = T_as*p_s;

  			position_msg.position.x = p_a(0,0);
  			position_msg.position.y = p_a(1,0);

		  	ROS_INFO("Center Point [%f], [%f]", p_a(0,0)/10, p_a(1,0)/10);

  			pose_pub.publish(position_msg);
  			
  			// Show images
  			cv::imshow("Circles ", orig_image);
        

  			cv::waitKey(2);
        //cv::waitKey(0);
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