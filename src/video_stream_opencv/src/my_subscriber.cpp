#include <ros/ros.h>
#include <image_transport/image_transport.h> //Includes everything needed to publish and subscribe images
// Allow us to display images using OpenCV's simple GUI capabilities
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
//#include <<sstream>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
/*
Callback function is called when a new image arrives on the "camera/image" topic
Although the image may have been sent in some arbitrary transport-specific message type, notice that 
the callback need only handle the normal "sensor_msgs/Image" type. All image encodings/decodings is
handled automagically for you
*/

{
  //Convert ROS message to an OpenCV Image with BGR pixel encoding, then show it in a display winow.
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  // Create an OpenCV display window
  ros::NodeHandle nh;
  cv::namedWindow("view");
  
  cv::startWindowThread();
  /*
  Create ImageTransport instance, intialize it with our NodeHandle. We use methods of ImageTransport to create
  image publishers and subscribers, much as we use methods of NodeHandle to create generic ROS publishers and
  subscribers
  */

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/simple_canny", 1, imageCallback);
  /*
  - Subscribe to the "camera/image" base topic
  - Actual ROS topic subscribed to depends on which transports is used. In the default case, "raw" transport,
    the topic is in fact "camera/image" with type 'sensor_msgs/Image.' 
  - ROS will call the ImageCallback function whenever a new image arrives. 
  - The second argument is the queue size
  - Different versions of the subscribe() function will allow you to specify a class member function, or 
    eve anything callable by a Boost.Function object  
  */
  
  // Dispose of our display window
  ros::spin();
  cv::destroyWindow("view");  
}
