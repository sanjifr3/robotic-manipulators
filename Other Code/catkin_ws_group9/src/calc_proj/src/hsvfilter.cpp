#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
#include <sstream>

using namespace cv;
using namespace std;

int main( int argc, char **argv )
{
    
    ros::init(argc, argv, "Calc_Pos_Data");
    ros::NodeHandle nh;

    ros::Publisher calc_data_pub = nh.advertise<std_msgs::Float32MultiArray>("Calc_Pos_Data", 1000);
    ros::Rate loop_rate(10);

    VideoCapture cap(0); //capture the video from web cam


    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"



    // Yellow Detection
    int iLowHyel = 0;
    int iHighHyel = 34;
    int iLowSyel = 135; 
    int iHighSyel = 185;
    int iLowVyel = 105;
    int iHighVyel = 164;

    // Green Detection
    int iLowHgre = 60;
    int iHighHgre = 84;
    int iLowSgre = 73; 
    int iHighSgre = 137;
    int iLowVgre = 49;
    int iHighVgre = 92;

    // Blue Detection
    int iLowHblu = 100;
    int iHighHblu = 117;
    int iLowSblu = 117; 
    int iHighSblu = 157;
    int iLowVblu = 96;
    int iHighVblu = 130;

    // Set Hough Parameters
    int ihough_low = 5;
    int ihough_high = 350;

    float theta = -1;
    


    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowHgre, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighHgre, 179);

    cvCreateTrackbar("LowS", "Control", &iLowSgre, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighSgre, 255);

    cvCreateTrackbar("LowV", "Control", &iLowVgre, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighVgre, 255);

    while (ros::ok())
    {

        std_msgs::Float32MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 3;
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label = "data";

        Mat imgOriginal;
        Mat imgHSV;
        Mat imgThresholdedyel;
        Mat imgThresholdedgre;
        Mat imgThresholdedblu;
        cv::vector<Vec3f> circlesyel;
        cv::vector<Vec3f> circlesgre;
        cv::vector<Vec3f> circlesblu;


        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        // Circle detection for yellow dot
        inRange(imgHSV, Scalar(iLowHyel, iLowSyel, iLowVyel), Scalar(iHighHyel, iHighSyel, iHighVyel), imgThresholdedyel); //Threshold the image
              
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholdedyel, imgThresholdedyel, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholdedyel, imgThresholdedyel, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholdedyel, imgThresholdedyel, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholdedyel, imgThresholdedyel, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        // Decreasing canny detection includes objects that are less circular, Decreasing center detection includes more
        HoughCircles(imgThresholdedyel, circlesyel, CV_HOUGH_GRADIENT, 1, imgThresholdedyel.rows/8, ihough_high, ihough_low, 0, 60);


        // ROS_INFO("Yellow: %d", circlesyel.size());

        // Draw the circles detected, useful for debugging
        for (size_t i = 0; i < circlesyel.size(); i++ )
        {
            Point center(cvRound(circlesyel[i][0]), cvRound(circlesyel[i][1]));
            int radius = cvRound(circlesyel[i][2]);
            // circle center
            circle(imgThresholdedyel, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(imgOriginal, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(imgThresholdedyel, center, radius, Scalar(150, 0, 0), 3, 8, 0);
            circle(imgOriginal, center, radius, Scalar(150, 0, 0), 3, 8, 0);
        }


        // Circle detection for green dot
        inRange(imgHSV, Scalar(iLowHgre, iLowSgre, iLowVgre), Scalar(iHighHgre, iHighSgre, iHighVgre), imgThresholdedgre); //Threshold the image
              
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholdedgre, imgThresholdedgre, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholdedgre, imgThresholdedgre, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        dilate(imgThresholdedgre, imgThresholdedgre, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholdedgre, imgThresholdedgre, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        // Decreasing canny detection includes objects that are less circular, Decreasing center detection includes more
        HoughCircles(imgThresholdedgre, circlesgre, CV_HOUGH_GRADIENT, 1, imgThresholdedgre.rows/8, ihough_high, ihough_low, 0, 35);


        // ROS_INFO("Green: %d", circlesgre.size());


        int pointgre [2] = {-1,-1};

        // Draw the circles detected, useful for debugging
        for (size_t i = 0; i < circlesgre.size(); i++ )
        {
            Point center(cvRound(circlesgre[i][0]), cvRound(circlesgre[i][1]));
            int radius = cvRound(circlesgre[i][2]);
            // circle center
            circle(imgThresholdedgre, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(imgOriginal, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(imgThresholdedgre, center, radius, Scalar(150, 0, 0), 3, 8, 0);
            circle(imgOriginal, center, radius, Scalar(150, 0, 0), 3, 8, 0);
            pointgre[0] = center.x;
            pointgre[1] = center.y;        
        }
        ROS_INFO("Green Point: %i, %i",pointgre[0],pointgre[1]);


        // Circle detection for blue dot
        inRange(imgHSV, Scalar(iLowHblu, iLowSblu, iLowVblu), Scalar(iHighHblu, iHighSblu, iHighVblu), imgThresholdedblu); //Threshold the image
              
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholdedblu, imgThresholdedblu, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholdedblu, imgThresholdedblu, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholdedblu, imgThresholdedblu, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholdedblu, imgThresholdedblu, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        // Decreasing canny detection includes objects that are less circular, Decreasing center detection includes more
        HoughCircles(imgThresholdedblu, circlesblu, CV_HOUGH_GRADIENT, 1, imgThresholdedblu.rows/8, ihough_high, ihough_low, 0, 35);


        // ROS_INFO("Blue: %d", circlesblu.size());

        int pointblu [2] = {-1,-1};

        // Draw the circles detected, useful for debugging
        for (size_t i = 0; i < circlesblu.size(); i++ )
        {
            Point center(cvRound(circlesblu[i][0]), cvRound(circlesblu[i][1]));
            int radius = cvRound(circlesblu[i][2]);
            // circle center
            circle(imgThresholdedblu, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(imgThresholdedblu, center, radius, Scalar(150, 0, 0), 3, 8, 0);
            pointblu[0] = center.x;
            pointblu[1] = center.y; 
        }
        
        if(pointblu[0] != -1 && pointgre[0] != -1)
        {
            theta = atan2((pointblu[1]-pointgre[1]), (pointblu[0]-pointgre[0]) );
        }

        // msg.data.push_back(1);
        msg.data.push_back((float)(pointgre[0]));
        msg.data.push_back((float)(pointgre[1]));
        msg.data.push_back(theta);


        ROS_INFO("Blue Point: %i, %i",pointblu[0],pointblu[1]);

        imshow("Original", imgOriginal); //show the original image
        imshow("Thresholded Image Yellow", imgThresholdedyel); //show the thresholded image
        imshow("Thresholded Image Green", imgThresholdedgre); //show the thresholded image
        imshow("Thresholded Image Blue", imgThresholdedblu); //show the thresholded image

            if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
           {
                cout << "esc key is pressed by user" << endl;
                break; 
           }
        calc_data_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

   return 0;

}