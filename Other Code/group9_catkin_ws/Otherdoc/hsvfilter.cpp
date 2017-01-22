#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
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
    int iLowSgre = 98; 
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

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowHyel, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighHyel, 179);

    cvCreateTrackbar("LowS", "Control", &iLowSyel, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighSyel, 255);

    cvCreateTrackbar("LowV", "Control", &iLowVyel, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighVyel, 255);

    while (true)
    {
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
            // circle outline
            circle(imgThresholdedyel, center, radius, Scalar(150, 0, 0), 3, 8, 0);
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


        int pointgre [2] = {0,0};

        // Draw the circles detected, useful for debugging
        for (size_t i = 0; i < circlesgre.size(); i++ )
        {
            Point center(cvRound(circlesgre[i][0]), cvRound(circlesgre[i][1]));
            int radius = cvRound(circlesgre[i][2]);
            // circle center
            circle(imgThresholdedgre, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(imgThresholdedgre, center, radius, Scalar(150, 0, 0), 3, 8, 0);
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

        int pointblu [2] = {0,0};

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
    }

   return 0;

}