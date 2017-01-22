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

ros::Publisher featurePublisher;
ros::Subscriber imageSubscriber;
ros::Subscriber commandSubscriber;
std::vector<geometry_msgs::Pose2D> poseList(NUMBER_OF_BLOCKS);

bool extract = true;

void extractFeature(cv::Mat src);

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
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void drawPoint( Mat img, Point center, const Scalar& color)
{
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         2,
         color,
         thickness,
         lineType );
}

void drawLine( Mat img, Point start, Point end)
{
  int thickness = 2;
  int lineType = 8;
  line( img,
        start,
        end,
        Scalar( 0, 0, 0 ),
        thickness,
        lineType );
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

cv::Point getAverage(std::vector<cv::Point> pointList)
{
  cv::Point avg(0,0);
  for (int i = 0; i < pointList.size(); i++) {
    avg.x += pointList[i].x;
    avg.y += pointList[i].y;
  }
  avg.x /= pointList.size();
  avg.y /= pointList.size();
  return avg;
}

double d22p(cv::Point pA, cv::Point pB)
{
  return (pB.x-pA.x)*(pB.x-pA.x) + (pB.y-pA.y)*(pB.y-pA.y);
}

double getAngle(std::vector<cv::Point> pointList)
{
  if (pointList.size() < 4) {
    return 0;
  }

  cv::Point pA = pointList[0];
  cv::Point pB = pointList[1];
  cv::Point pC = pointList[2];

  if(d22p(pA,pB) > d22p(pB,pC)) {
    return atan2(pB.y - pA.y, pB.x - pA.x);
  } else {
    return atan2(pC.y - pB.y, pC.x - pB.x);
  }
}

geometry_msgs::Pose2D getShortestDistance(std::vector<geometry_msgs::Pose2D> poseList, cv::Point target)
{
  double max = 999999;
  int index = 0;
  for (int i = 0; i < poseList.size(); i ++) {
    cv:Point p(poseList[i].x,poseList[i].y);
    double d = d22p(target, p);
    if (d < max) {
      d = max;
      index = i;
    }
  }
  return poseList[index];
}

/**
 * Simple shape detector program.
 * It loads an image and tries to find simple shapes (rectangle, triangle, circle, etc) in it.
 * This program is a modified version of `squares.cpp` found in the OpenCV sample dir.
 */
void extractFeature(cv::Mat src)
{
  // Convert to grayscale
  cv::Mat gray;
  cv::cvtColor(src, gray, CV_BGR2GRAY);

  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat bw;
  cv::Canny(gray, bw, 0, 50, 5);

  // Find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> approx;
  cv::Mat dst = src.clone();

  for (int i = 0; i < contours.size(); i++)
  {
    cout << "Contour: " << i << endl;

    // Approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.03, true);

    if (approx.size() == 4 && 
      fabs(contourArea(Mat(approx))) > 100 &&
      isContourConvex(Mat(approx)))
    {
      cout << "Approx: ";
      for (int j = 0; j < approx.size(); j++) {
        cout << approx[j] << " ";
        drawPoint(dst, approx[j], Scalar( j*20+100, j*20+100, j*20+100));
      }
      cout << endl;

      double maxCosine = 0;

      for( int j = 2; j < 5; j++ )
      {
          double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
          maxCosine = MAX(maxCosine, cosine);
      }

      if( maxCosine < 0.35 ) {
        stringstream ss;
        ss << i;
        string str = ss.str();
        setLabel(dst, str, contours[i]);


        cv::Point centroid = getAverage(approx);
        drawPoint(dst, centroid, Scalar(100,100,100));

        double angle = getAngle(approx);
        cout << "Angle: " << angle << endl;

        geometry_msgs::Pose2D featurePose;
        featurePose.x = centroid.x;
        featurePose.y = centroid.y;
        featurePose.theta = M_PI - angle;
        cout << "Centroid: " << featurePose.x << ", " << featurePose.y << endl;

        poseList.push_back(featurePose);
      }
    }
  }

  cv::Point target(400,0);
  geometry_msgs::Pose2D featurePose = getShortestDistance(poseList, target);
  cv::Point pose(featurePose.x,featurePose.y);
  drawLine(dst, target, pose);

  featurePose.x -= target.x;
  featurePose.y -= target.y;

  cv::imshow("dst", dst);
  featurePublisher.publish(featurePose);
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extraction");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  featurePublisher = nh.advertise<geometry_msgs::Pose2D>("/feature/pose2D", 1, true);
  imageSubscriber = nh.subscribe("/camera/image_raw", 1, imageCallback);  
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
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"

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

            
            
            cv::Mat src_gray;
            cv::Mat src = cv_ptr -> image;
            cv::Mat orig_image = src.clone();

            
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

   // Convert to grayscale
  cv::Mat gray;
  cv::cvtColor(src, gray, CV_BGR2GRAY);

  // Use Canny instead of threshold to catch squares with gradient shading
  cv::Mat bw;
  cv::Canny(gray, bw, 0, 50, 5);

  // Find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> approx;
  cv::Mat dst = src.clone();

  for (int i = 0; i < contours.size(); i++)
  {
    cout << "Contour: " << i << endl;

    // Approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.03, true);

    if (approx.size() == 4 && 
      fabs(contourArea(Mat(approx))) > 100 &&
      isContourConvex(Mat(approx)))
    {
      cout << "Approx: ";
      for (int j = 0; j < approx.size(); j++) {
        cout << approx[j] << " ";
        drawPoint(dst, approx[j], Scalar( j*20+100, j*20+100, j*20+100));
      }
      cout << endl;

      double maxCosine = 0;

      for( int j = 2; j < 5; j++ )
      {
          double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
          maxCosine = MAX(maxCosine, cosine);
      }

      if( maxCosine < 0.35 ) {
        stringstream ss;
        ss << i;
        string str = ss.str();
        setLabel(dst, str, contours[i]);


        cv::Point centroid = getAverage(approx);
        drawPoint(dst, centroid, Scalar(100,100,100));

        double angle = getAngle(approx);
        cout << "Angle: " << angle << endl;

        geometry_msgs::Pose2D featurePose;
        featurePose.x = centroid.x;
        featurePose.y = centroid.y;
        featurePose.theta = M_PI - angle;
        cout << "Centroid: " << featurePose.x << ", " << featurePose.y << endl;

        poseList.push_back(featurePose);
      }
    }
  }

        cv::Point target(400,0);
        geometry_msgs::Pose2D featurePose = getShortestDistance(poseList, target);
        cv::Point pose(featurePose.x,featurePose.y);
        drawLine(dst, target, pose);

        featurePose.x -= target.x;
        featurePose.y -= target.y;

        cv::imshow("dst", dst);
        featurePublisher.publish(featurePose);












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
            //waitKey(2);
            waitKey(0);

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

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int main(int argc, char** argv)
{	
	ros::init(argc,argv,"Image_Processor");
    BlobDetector ic;

    ros::spin();

    return 0;
}