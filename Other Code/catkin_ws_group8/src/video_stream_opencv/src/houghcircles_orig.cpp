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

#include <geometry_msgs/Pose2D.h>
#include <string>
#include <ros/console.h>
ros::Publisher pub;
 
using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;




ros::Publisher pub1 ;
ros::Publisher pub2 ;
ros::Publisher pub3 ;





namespace
{
    // windows and trackbars name
    const std::string windowName = "Hough Circle Detection";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
    const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";

    // initial and max values of the parameters of interests.
    //const int cannyThresholdInitialValue = 90;
    //const int accumulatorThresholdInitialValue = 38;
    const int cannyThresholdInitialValue = 85;
    const int accumulatorThresholdInitialValue = 40;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;

    void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/10, cannyThreshold, accumulatorThreshold, 0, 30 );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        geometry_msgs::Pose2D XXX;
        geometry_msgs::Pose2D YYY;
        geometry_msgs::Pose2D WWW;
        if ( circles.size()==3 )
        {
          for( size_t i = 0; i < circles.size(); i++ )
          {
              Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
              int radius = cvRound(circles[i][2]);
              // circle center
              circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
              // circle outline
              circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
          }
          double dcircle1x = circles[0][0];
          double dcircle1y = circles[0][1];
          double dcircle2x = circles[1][0];
          double dcircle2y = circles[1][1];
          double dcircle3x = circles[2][0];
          double dcircle3y = circles[2][1];

          if (dcircle1x < dcircle2x && dcircle1x < dcircle3x)
          {
            circles[0][0]=dcircle1x;
            circles[0][1]=dcircle1y;
            if(dcircle2x < dcircle3x)
            {
              circles[1][0]=dcircle2x;
              circles[1][1]=dcircle2y;
              circles[2][0]=dcircle3x;
              circles[2][1]=dcircle3y;
            }
            else
            {
              circles[1][0]=dcircle3x;
              circles[1][1]=dcircle3y;
              circles[2][0]=dcircle2x;
              circles[2][1]=dcircle2y;
            }
          }
          else if (dcircle2x < dcircle1x && dcircle2x < dcircle3x)
          {
            circles[0][0]=dcircle2x;
            circles[0][1]=dcircle2y;
            if(dcircle1x < dcircle3x)
            {
              circles[1][0]=dcircle1x;
              circles[1][1]=dcircle1y;
              circles[2][0]=dcircle3x;
              circles[2][1]=dcircle3y;
            }
            else
            {
              circles[1][0]=dcircle3x;
              circles[1][1]=dcircle3y;
              circles[2][0]=dcircle1x;
              circles[2][1]=dcircle1y;
            }
          }
          else
          {
            circles[0][0]=dcircle3x;
            circles[0][1]=dcircle3y;
            if(dcircle1x < dcircle2x)
            {
              circles[1][0]=dcircle1x;
              circles[1][1]=dcircle1y;
              circles[2][0]=dcircle2x;
              circles[2][1]=dcircle2y;
            }
            else
            {
              circles[1][0]=dcircle2x;
              circles[1][1]=dcircle2y;
              circles[2][0]=dcircle1x;
              circles[2][1]=dcircle1y;
            }
          }

          XXX.x = circles[0][0];
          XXX.y = circles[0][1];
          YYY.x = circles[1][0];
          YYY.y = circles[1][1];
          WWW.x = circles[2][0];
          WWW.y = circles[2][1];
          //cout<<XXX<<endl;
          //cout<<XXX.y<<endl;
          pub1.publish(XXX);
          pub2.publish(YYY);
          pub3.publish(WWW);
          // shows the results
          imshow( windowName, display);
        }
    }
}

static const char WINDOW[] = "Image window";


class hohohoughcircles
{
    ros::NodeHandle nh_;
ros::NodeHandle n;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;

public:
 hohohoughcircles()
    : it_(nh_)
  {
 
     image_sub_ = it_.subscribe("/webcam/image_raw", 1, &hohohoughcircles::imageCb, this);
     image_pub_= it_.advertise("/camera/image_processed",1);

  pub1 = n.advertise<geometry_msgs::Pose2D>("/circle_coords1",1);
  pub2 = n.advertise<geometry_msgs::Pose2D>("/circle_coords2",1);
  pub3 = n.advertise<geometry_msgs::Pose2D>("/circle_coords3",1);
 
 
  }
 
  ~hohohoughcircles()
  {
    cv::destroyWindow(WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& original_image)
  {

    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;

        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    
 
    Mat src = cv_ptr->image;
    Mat src_gray;


    // Convert it to gray
    cvtColor( src, src_gray, COLOR_BGR2GRAY );

    // Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );



//
    //declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;

    // create the main window, and attach the trackbars
    namedWindow( windowName, WINDOW_AUTOSIZE );
    //createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
    //createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);

    // infinite loop to display
    // and refresh the content of the output image
    // until the user presses q or Q
    int key = 0;
    //while(key != 'q' && key != 'Q')
    for(int i=0;i<10;i++)
    {
        // those paramaters cannot be =0
        // so we must check here
        cannyThreshold = std::max(cannyThreshold, 1);
        accumulatorThreshold = std::max(accumulatorThreshold, 1);

        //runs the detection, and update the display
        HoughDetection(src_gray, src_gray, cannyThreshold, accumulatorThreshold);

        // get user key
        key = waitKey(10);
    }
//


      cvWaitKey(2);   
 
}
};

 
 
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hohohoughcircles");
  hohohoughcircles ic;







  ros::spin();
 
  return 0;
}



