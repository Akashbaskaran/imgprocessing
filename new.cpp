#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/String.h"
#include "sbot_msg/TargetData.h"
#include <sstream>
#include "opencv2/core/core.hpp"
#include <math.h>  

#define PI 3.14159265

 int iLowH = 0;
 int iHighH = 179;

 int iLowS = 0; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection 1";


std::vector<cv::Point2f> get_positions(cv::Mat& imaged)
{
    if (imaged.channels() > 1)
    {
        std::cout << "get_positions: !!! Input image must have a single channel" << std::endl;
        return std::vector<cv::Point2f>();
    }

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(imaged, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Approximate contours to polygons and then get the center of the objects
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());
    for (unsigned int i = 0; i < contours.size(); i++ )
    {
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 5, true );
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
    }

    return center;
}   

class Edge_Detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher test;
  
public:
  Edge_Detector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    //For kinect
  // image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,&Edge_Detector::imageCb, this); 
    //For Usb camera
      image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&Edge_Detector::imageCb, this); 
      image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
      test = nh_.advertise<sbot_msg::TargetData>("test_topic",10);
    cv::namedWindow(OPENCV_WINDOW);
    
  }

  ~Edge_Detector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


 void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){

	detect_edges(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	}
  }
  void detect_edges(cv::Mat img)

{




 namedWindow("Blue", CV_WINDOW_AUTOSIZE); //create a window called "Control"
 createTrackbar("LowH", "Blue", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Blue", &iHighH, 179);
 createTrackbar("LowS", "Blue", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Blue", &iHighS, 255);

 createTrackbar("LowV", "Blue", &iLowV, 255); //Value (0 - 255)
 createTrackbar("HighV", "Blue", &iHighV, 255);

Rect box;
  Mat imgHSV;
  Mat imgThresholded; 
  
  

        box.width=600;
        box.height=400;
        box.x=0;
        box.y=0;
        Mat crop(img,box);

cvtColor(crop, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  //for changing values in variable trackbar
 inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
// inRange(imgHSV, Scalar(0, 111, 143), Scalar(179, 255, 255), imgThresholded);  

 //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 10)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  
  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 2)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


cv::imshow(OPENCV_WINDOW_1, imgThresholded);



 Mat dist_8u;

//define which image to convert to
    imgThresholded.convertTo(dist_8u, CV_8U);

    // Find total markers
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    findContours(dist_8u, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    // Find the rotated rectangles
    vector<RotatedRect> minRect( contours.size() );
  for( size_t i = 0; i < contours.size(); i++ )
    {
        minRect[i] = minAreaRect( Mat(contours[i]) );
    }


    RNG rng(12345);
    for( size_t i1 = 0; i1< contours.size(); i1++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // contour
       // drawContours( img, contours, static_cast<int>(i), color, 1, 8, vector<Vec4i>(), 0, Point() );
        // rotated rectangle
        Point2f rect_points[4]; minRect[i1].points( rect_points );
        for( int j = 0; j < 4; j++ )
         line( img, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
      

    }

   
for( size_t p = 0; p< 1; p++ )
{
 cv::Moments mu = cv::moments(contours[p]);
 cv::Point centroid = cv::Point (mu.m10/mu.m00 , mu.m01/mu.m00);
  circle(img,centroid, 2, Scalar(00,00,255), -1);
sbot_msg::TargetData msg;
	msg.x = centroid.x;
	msg.y = centroid.y;
	msg.color = 0;
	test.publish(msg);
}


cv::imshow(OPENCV_WINDOW, img);
cv::waitKey(3);


	
};
 

};


int main(int argc, char** argv)
{ ros::init(argc, argv, "Kinect_Data");
  ros::init(argc, argv, "pub2");
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}



