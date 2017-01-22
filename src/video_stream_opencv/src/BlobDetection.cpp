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
//Eigen::MatrixXf p_s(4,1);
//Eigen::MatrixXf p_a(4,1);

class BlobDetector
{
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pose_pub_;
	//ros::Publisher pose_pub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	std_msgs::String msg;
	
	//SimpleBlobDetector blob;
	vector<KeyPoint> centre_pts;
	//vector<KeyPoint> keypoints;
  	geometry_msgs::Pose pos_msgs;
  	//geometry_msgs::Pose position_msg;

	public:
		BlobDetector()
			: it_(nh_)
		{
     		image_sub_ = it_.subscribe("/webcam/image_raw", 1, &BlobDetector::imageCb, this);
     		image_pub_ = it_.advertise("/camera/image_processed",1);
     		pose_pub_ = n.advertise<geometry_msgs::Pose>("/position", 5);
     		//pose_pub = n.advertise<geometry_msg::Pose>("/position", 20);
		}

		~BlobDetector()
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

            /*
            
            cv::Mat src_gray;
            cv::Mat src = cv_ptr -> image;
            cv::Mat orig_image = src.clone();

            */
        /*

            // http://stackoverflow.com/questions/8076889/tutorial-on-opencv-simpleblobdetector

            //Convert to grayscale
            cvtColor(src,src_gray,CV_BGR2GRAY);

            //Apply Gaussian blur to reduce noise and avoid false circle detection
            GaussianBlur (src_gray, src_gray, Size(9,9),2,2);


            // Set up the parameters
            cv::SimpleBlobDetector::Params params;
            params.minDistBetweenBlobs = 50.0f;
            params.filterByInertia = false;
            params.filterByConvexity = false;
            params.filterByColor = false;
            params.filterByCircularity = true;
            params.filterByArea = true;
            params.minArea = 20.0f;
            params.maxArea = 500.0f;
            params.minCircularity = 0.75;
            params.maxCircularity = 0.8;

            // Set up and create the detector using the parameters specified
            cv::SimpleBlobDetector blob_detector(params);

            // Detect!
            vector <cv::KeyPoint> keypoints;
            blob_detector.detect(src_gray,keypoints);

            // Extract the x y coordinates from the keypoints
            for (int i = 0; i < keypoints.size(); i++)
            {
                float X = keypoints[i].pt.x;
                float Y = keypoints[i].pt.y;
            }

            // Draw detected circles
            Mat im_with_keypoints;
            drawKeypoints( src, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            // Show blobs
            imshow("keypoints", im_with_keypoints);
            cv::waitKey(0);

        */

        /*

            //http://www.learnopencv.com/blob-detection-using-opencv-python-c/

            //Convert to grayscale
            cvtColor(src,src_gray,CV_BGR2GRAY);

            //Apply Gaussian blur to reduce noise and avoid false circle detection
            GaussianBlur (src_gray, src_gray, Size(9,9),2,2);            

            cv::SimpleBlobDetector::Params params;
            params.minThreshold = 5;
            params.maxThreshold = 50;

            params.filterByArea = true;
            params.minArea = 0;
            params.maxArea = 50;

            params.filterByCircularity = true;
            params.minCircularity = 0.75;
            params.maxCircularity = 0.8;

            // Filter by Convexity
            params.filterByConvexity = false;
            //params.minConvexity = 0.87;

            // Filter by Inertia
            params.filterByInertia = false;
            //params.minInertiaRatio = 0.01;

            // Storage for blobs
            vector<KeyPoint> keypoints;

            // Set up blob detector with parameters
            cv::SimpleBlobDetector blob_detector(params);

            // Draw detected blobs as red circles
            Mat im_with_keypoints;
            drawKeypoints( src, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            
            //Show Bloba
            imshow("keypoints", im_with_keypoints );
            waitKey(0);
        
        */













            //Read Image
            //Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );
            //std::string path_image{argv[1]}

            // Setup SimpleBlobDetector parameters.
            cv::SimpleBlobDetector::Params params;

            // Minimum distance between blocks
                //params.minDistBetweenBlobs = 50.0f;

        	// Change thresholds
            	// Convert the source images to several binary images by thresholding the source image with thresholds starting at minThreshold.
            	// These thresholds are incremented by thresholdStep until maxThreshold. So the first threshold is minThreshold, the second
            	// is minThreshold + thresholdStep, the third is minThreshold + 2 x thresholdStep, and so on.
        	params.minThreshold = 10;
        	params.maxThreshold = 200;

	        // Filter by Area
	            // Filter based on size, e.g. minArea = 100 will filter out blobs that have less than 100 pixels
            params.filterByArea = true;
            params.minArea = 100;
            params.maxArea = 500;

	        // Filter by Circularity
	            // Measures how close to a circle is the blob. Circle has a circularity of 1, and a square has a circularity of 0.785
            params.filterByCircularity = true;
            params.minCircularity = 0.700;
            params.maxCircularity = 0.850;

	        // Filter by Convexity
	            // Convexity is defined as the (Area of the Blob / Area of it's convex hull). Now, Convex Hull of a shape is the tightest convex
	            // shape that completely encloses the shape. To filter by convexity, set filterByConvexity = 1, followed by setting
	            // 0 <= minConvexity <= 1 and maxConvexity (<= 1)
            params.filterByConvexity = false;
	            //params.minConvexity = 0.87;

	        // Filter by Inertia
	            // Measures how elongated a shape is. E.g. for a circle, this value is 1, for an ellipse it is between 0 and 1, and for a line it is
	            // 0. To filter by inertia ratio, set filterByInertia = 1, and set <= minInertiaRatio <= 1 and maxInertiaRatio (<= 1) appropriately
            params.filterByInertia = true;
            params.minInertiaRatio = 0.01;

	        // Filter by Color
	            //params.filterByColor = false;

	        // Set up detector with parameters
            cv::SimpleBlobDetector blob_detector(params);
	            // Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

            cv::Mat bgr_image = cv_ptr -> image;
            cv::Mat orig_image = bgr_image.clone();
            cv::Mat gray_image;
            cv::medianBlur(bgr_image, bgr_image, 3);
            cv::Mat hsv_image;
            cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);        
            //cv::cvtColor(bgr_image,gray_image, CV_GRAY2BGR);
            // Detect blobs
            //detector->detect(hsv_image, centre_pts);
            blob_detector.detect(hsv_image, centre_pts);

            // Draw detected blobs as red circles.
            // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
            // the size of the circle corresponds to the size of blob

            cv::Mat im_with_keypoints;
            drawKeypoints( hsv_image, centre_pts, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            // Show blobs
            imshow("Centre Points", im_with_keypoints );
            waitKey(2);
            //waitKey(0);

            // extract the x y coordinates of the keypoints:

            for (int i=0; i<centre_pts.size(); i++)
            {
                float I = centre_pts[i].pt.x;
                float J = centre_pts[i].pt.y;
            }

            ROS_INFO("Center Point [%f], [%f]", I, J);

            pose_pub_.publish(pos_msgs);

            

        }


};

int main(int argc, char** argv)
{	
	ros::init(argc,argv,"Image_Processor");
    BlobDetector ic;

    ros::spin();

    return 0;
}