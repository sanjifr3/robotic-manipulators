/*
Copyback (c) 2011, Siddhant Ahuja (Sid)
All backs reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyback
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyback
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYback HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYback HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This program contains the code for calculating the checkerboard pose
*/

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

// Messages
#include "checkerBoardPoseCalculate/checkerBoardPose.h"

#define limitRange 20.0

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
//pitch yaw roll
//x y z roll pitch yaw
//-z x -y  

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

ros::Publisher pubCam;
checkerBoardPoseCalculate::checkerBoardPose msgPose;

image_geometry::PinholeCameraModel pCam; 

std::vector<cv::Point2f> corners;
std::vector<cv::Point2f> imgpts;
double fR3[3], fT3[3];
cv::Mat rvec(3, 1, CV_64FC1, fR3);
cv::Mat tvec(3, 1, CV_64FC1, fT3);
bool foundCheckerboard;
int nX = 8 ;
int nY = 6 ;
double squareSize = 0.0254;

cv::Mat gray_image;

#define RTOD(a) ((a) * 180.0 / M_PI )

cv::Mat getObjPoints( int x, int y, double size )
{
	bool xodd = x % 2 == 1; bool yodd = x % 2 == 1;

	std::vector<cv::Point3f> corners;

	for( int i = 0; i < y; i++ )
	{
		float ycoord = size*(-y/2+i);
		if( !yodd ) ycoord += size/2.0;

		for( int j = 0; j < x; j++ )
		{
			float xcoord = size*(-x/2+j);
			if( !xodd ) xcoord += size/2.0;
			cv::Point3f p;
			p.x = xcoord;
			p.y = ycoord;
			p.z = 0;
			corners.push_back(p);
		}
	}	
	cv::Mat ret(cv::Size(x*y,1), CV_32FC3, (void*)&corners[0]);
	return ret; 
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image, const sensor_msgs::CameraInfoConstPtr& infomsg)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	foundCheckerboard = cv::findChessboardCorners(cv_ptr->image, cvSize(nX,nY), corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS  );

	if(foundCheckerboard)
	{
		// Get subpixel accuracy on those corners
		cv::cvtColor( cv_ptr->image, gray_image, CV_BGR2GRAY );
		cv::cornerSubPix( gray_image, corners, cvSize( 11, 11 ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		cv::Mat cornersmat( cv::Size(corners.size(), 1), CV_32FC2, (void*)&corners[0] );
	 	cv::drawChessboardCorners(cv_ptr->image, cvSize(nX,nY), cornersmat, true );
		pCam.fromCameraInfo(infomsg);
		cv::Mat objpts = getObjPoints(nX,nY,squareSize);
		
		cv::solvePnP( objpts, cornersmat, pCam.intrinsicMatrix(), pCam.distortionCoeffs(), rvec, tvec );
		
		double rx = fR3[0] + M_PI, ry = fR3[1], rz = fR3[2];
		
    // make a rostopic here !!!!

    //restrict the range to 20 meters
    if (fT3[0] >=limitRange || fT3[0]<=-limitRange || fT3[1] >=limitRange || fT3[1] <= -limitRange || fT3[2] >= limitRange || fT3[2] <=-limitRange)
    {
      //do nothing
    }
    else
    {
      msgPose.header.stamp = ros::Time::now();
      msgPose.tx = fT3[0];
      msgPose.ty = fT3[1];		
      msgPose.tz = fT3[2];

      msgPose.rx = rx;
      msgPose.ry = ry;		
      msgPose.rz = rz;

      pubCam.publish(msgPose);
  
    }
		// ROS_INFO( "tx: (%0.2f,%0.2f,%0.2f) rx: (%0.2f,%0.2f,%0.2f)", fT3[0],fT3[1], fT3[2], fR3[0],fR3[1],fR3[2]);
	}
	//Display the image using OpenCV
	 cv::imshow(WINDOW, cv_ptr->image);
	
	
	
	//Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	char c = cvWaitKey(3);
	if( c == 32)
	{
		std::cout << "Saving image to /home/me547/image1.jpg" << std::endl;
		imwrite( "/home/me547/image1.jpg", cv_ptr->image);
	} 
	
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{

        ros::init(argc, argv, "CheckerBoardPoseCalculate");

        ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
        
	//OpenCV HighGUI call to create a display window on start-up.
	// cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

        image_transport::CameraSubscriber sub = it.subscribeCamera("/image_raw", 1, imageCallback);
	
	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	
     
  //Advertise publishable topics
	pubCam = nh.advertise<checkerBoardPoseCalculate::checkerBoardPose>("/camPose", 1, false);

        ros::spin();
	return 0;
}

