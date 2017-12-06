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
/*
 int iLowH2 = 0;
 int iHighH2 = 179;

 int iLowS2 = 0; 
 int iHighS2 = 255;

 int iLowV2 = 0;
 int iHighV2 = 255;

 int iLowH3 = 0;
 int iHighH3 = 179;

 int iLowS3 = 0; 
 int iHighS3 = 255;

 int iLowV3 = 0;
 int iHighV3 = 255;
 
int iLowH4 = 0;
 int iHighH4 = 179;

 int iLowS4 = 0; 
 int iHighS4 = 255;

 int iLowV4 = 0;
 int iHighV4 = 255;
*/

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection 1";

/*static const std::string OPENCV_WINDOW_2 = "Edge Detection 2";
static const std::string OPENCV_WINDOW_3 = "Edge Detection 3";
static const std::string OPENCV_WINDOW_4 = "Edge Detection 4";
*/

/* get_positions: a function to retrieve the center of the detected blobs.
 * largely based on OpenCV's "Creating Bounding boxes and circles for contours" tutorial.
 */

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

// namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
/*namedWindow("Control2", CV_WINDOW_AUTOSIZE); //create a window called "Control"
 namedWindow("Control3", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  namedWindow("Control4", CV_WINDOW_AUTOSIZE); //create a window called "Control"
*/

/*
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);
 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);
*/

/* createTrackbar("LowH2", "Control2", &iLowH2, 179); //Hue (0 - 179)
 createTrackbar("HighH2", "Control2", &iHighH2, 179);
 createTrackbar("LowS2", "Control2", &iLowS2, 255); //Saturation (0 - 255)
 createTrackbar("HighS2", "Control2", &iHighS2, 255);

 createTrackbar("LowV2", "Control2", &iLowV2, 255); //Value (0 - 255)
 createTrackbar("HighV2", "Control2", &iHighV2, 255);


 createTrackbar("LowH3", "Control3", &iLowH3, 179); //Hue (0 - 179)
 createTrackbar("HighH3", "Control3", &iHighH3, 179);
 createTrackbar("LowS3", "Control3", &iLowS3, 255); //Saturation (0 - 255)
 createTrackbar("HighS3", "Control3", &iHighS3, 255);

 createTrackbar("LowV3", "Control3", &iLowV3, 255); //Value (0 - 255)
 createTrackbar("HighV3", "Control3", &iHighV3, 255);



 createTrackbar("LowH4", "Control4", &iLowH4, 179); //Hue (0 - 179)
 createTrackbar("HighH4", "Control4", &iHighH4, 179);
 createTrackbar("LowS4", "Control4", &iLowS4, 255); //Saturation (0 - 255)
 createTrackbar("HighS4", "Control4", &iHighS4, 255);

 createTrackbar("LowV4", "Control4", &iLowV4, 255); //Value (0 - 255)
 createTrackbar("HighV4", "Control4", &iHighV4, 255);
*/
/*////////////Image Cropping/////////
cv::Mat imgcropped;
imgcropped = img(Rect(127,0,500,330));
//////////////////////////////////*/
Rect box;
 
  Mat imgHSV;Mat imgThresholded; 
  Mat imgHSV2;Mat imgThresholded2;
  Mat imgHSV3;Mat imgThresholded3;
  Mat imgHSV4;Mat imgThresholded4;

        box.width=450;
        box.height=375;
        box.x=0;
        box.y=0;
        Mat crop(img,box);

cvtColor(crop, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

cvtColor(crop, imgHSV2, COLOR_BGR2HSV);
cvtColor(crop, imgHSV3, COLOR_BGR2HSV);
cvtColor(crop, imgHSV4, COLOR_BGR2HSV);
  
  //for camera white
  // inRange(imgHSV, Scalar(123, 51, 255), Scalar(179, 255, 255), imgThresholded); //Threshold the image
//for kinect
//for changing values in variable trackbar
  //inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
  //inRange(imgHSV2, Scalar(iLowH2, iLowS2, iLowV2), Scalar(iHighH2, iHighS2, iHighV2), imgThresholded2); //Threshold the image 
 // inRange(imgHSV3, Scalar(iLowH3, iLowS3, iLowV3), Scalar(iHighH3, iHighS3, iHighV3), imgThresholded3); //Threshold the image 
 // inRange(imgHSV4, Scalar(iLowH4, iLowS4, iLowV4), Scalar(iHighH4, iHighS4, iHighV4), imgThresholded4); //Threshold the image 
   
//for fixed values in kinect
inRange(imgHSV, Scalar(0, 0, 143), Scalar(179, 255, 255), imgThresholded); 
//inRange(imgHSV2, Scalar(0, 29, 161), Scalar(68, 255, 255), imgThresholded2); //Threshold the image 
//inRange(imgHSV3, Scalar(98,62, 120), Scalar(179, 255, 217), imgThresholded3); //Threshold the image 
//inRange(imgHSV4, Scalar(25,29, 126), Scalar(118, 170, 255), imgThresholded4); //Threshold the image 

 //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(20, 2)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );  

  erode(imgThresholded3, imgThresholded3, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded3, imgThresholded3, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );  

  erode(imgThresholded4, imgThresholded4, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded4, imgThresholded4, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );  

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(15, 2)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(15, 2)) );

  dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(15, 5)) ); 
  erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
 
  dilate( imgThresholded3, imgThresholded3, getStructuringElement(MORPH_ELLIPSE, Size(15, 5)) ); 
  erode(imgThresholded3, imgThresholded3, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  
  dilate( imgThresholded4, imgThresholded4, getStructuringElement(MORPH_ELLIPSE, Size(15, 5)) ); 
  erode(imgThresholded4, imgThresholded4, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

//For one robot image moments
   //vector<vector<Point> > contours0;
   //findContours( imgThresholded, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

   //contours.resize(contours0.size());
//cv::imshow(OPENCV_WINDOW_1, imgThresholded);
//cv::imshow(OPENCV_WINDOW_2, imgThresholded2);
//cv::imshow(OPENCV_WINDOW_3, imgThresholded3); 
//cv::imshow(OPENCV_WINDOW_4, imgThresholded4); 


  // Retrieve a vector of points with the (x,y) location of the objects
     //   std::vector<cv::Point2f> points = get_positions(imgThresholded);

  // Draw a small green circle at those locations for educational purposes
        //for (unsigned int i = 0; i < points.size(); i++)
        //    cv::circle(img, points[i], 2, Scalar(00,00,255), -1);

       // cv::imshow(OPENCV_WINDOW, img);


cv::imshow(OPENCV_WINDOW_1, imgThresholded);
/*

//  Moments oMoments = moments(imgThresholded);
  Moments bMoments = moments(imgThresholded2);
  Moments cMoments = moments(imgThresholded3);
  Moments dMoments = moments(imgThresholded4);
 
  //double dM01 = oMoments.m01;
  //double dM10 = oMoments.m10;
  //double dArea = oMoments.m00;

  double bM01 = bMoments.m01;
  double bM10 = bMoments.m10;
  double bArea = bMoments.m00;

  double cM01 = cMoments.m01;
  double cM10 = cMoments.m10;
  double cArea = cMoments.m00;

  double dM01 = dMoments.m01;
  double dM10 = dMoments.m10;
  double dArea = dMoments.m00;


  // if the area <= 10000, I consider that the there are no objects in the image and it's because of the noise, the area is not zero 
 // if (dArea > 10000)
//  {
   //calculate the position of the ball
   int posdX = (dM10 / dArea);
   int posdY = dM01 / dArea;    
   int posbX= (bM10/ bArea);
   int posbY= bM01/ bArea; 
   int poscX= (cM10/ cArea);
   int poscY= cM01/ cArea;    

//float result;
//float m;
//m=(posX-posbX)/(posY-posbY);
//result = (atan(m)*180)/PI;
//red,purple,
float desX, desY, desbX,desbY, descX, descY, bou1,bou2,bou3,bou4;
desX=250;
desY=250;
desbX=230;
desbY=230;
descX=270;
descY=270;
int possrX=5;
int possrY=5;
int posfrX=450;
int posfrY=375;
bou1=possrX+25;
bou2=possrY+25;
bou3=posfrX-25;
bou4=posfrY-25;

float posX, posY;
posX=points[0].x;
posY=points[0].y;
float angle = (atan2(posY - poscY, posX - poscX)*180/PI);
//line(img, points[0], Point (poscX,poscY), Scalar(255,255,255), 1, 8,0);        
circle(img,Point (poscX,poscY), 2, Scalar(255,00,255), -1); 
circle(img,Point (poscX,poscY), 10, Scalar(0,255,255), 2); 
circle(img,Point (posdX,posdY), 2, Scalar(255,00,255), -1); 
circle(img,Point (posdX,posdY), 10, Scalar(0,255,255), 2); 


for(int h=0;h<points.size(); h++)
{
circle(img,points[h], 3, Scalar(00,00,255), -1); 
circle(img,points[h], 25, Scalar(0,255,0), 2);
}
//line(img, points[0], Point (poscX,poscY), Scalar(255,255,00), 2, 8,0);
line(img, Point (posbX,posbY), Point (450,posbY), Scalar(0,255,255), 1, 8,0);
line(img, Point (posbX,posbY), Point (posbX,375), Scalar(0,255,255), 1, 8,0);
line(img, Point (posbX,posbY), Point (5,posbY), Scalar(0,255,255), 1, 8,0);
line(img, Point (posbX,posbY), Point (posbX,5), Scalar(0,255,255), 1, 8,0);

circle(img,Point (posbX,posbY), 2, Scalar(00,00,255), -1);

rectangle(img, Point (possrX,possrY), Point (posfrX,posfrY), Scalar(191,191,191),1, 8, 0);
rectangle(img, Point (desbX,desbY), Point (descX,descY), Scalar(191,191,191),2, 8, 0);
rectangle(img, Point (bou1,bou2), Point (bou3,bou4), Scalar(0,255,0),1, 8, 0);
int no,obj;


*/
/*if(((abs(posX-450))<25) || ((abs(posX-0))<25) || ((abs(posY-375))<25) || ((abs(posY-0))<25))
{no=1;}
else
{no=0;}

if(((abs(posX-250))<20) && ((abs(posY-250))<250))
{obj=1;}
else
{obj=0;}
*/
//ros::NodeHandle n1;
//ros::Rate loop_rate(2);
/*ros::Publisher chatter_pub = n1.advertise<std_msgs::String>("Posxy", 1000);

std_msgs::String msg1;//msg,msg1,msg2,msg3,msg4;
std::stringstream ss1;//ss,ss1,ss2,ss3,ss4;
ss1<<"theta" << posX<<" "<<posY<<" "<<angle;
msg1.data=ss1.str();
 
ROS_INFO("%s", msg1.data.c_str()); 


chatter_pub.publish(msg1);*/
/*
ros::NodeHandle n2;
ros::Publisher chatter_Status = n2.advertise<std_msgs::String>("Status", 1000);
std_msgs::String msg2;
std::stringstream ss2;     //ss2,ss3,ss4;
ss2<<no;
msg2.data =ss2.str();
ROS_INFO("%s", msg2.data.c_str());
chatter_Status.publish(msg2);


ros::NodeHandle n3;
ros::Publisher chatter_Dest = n3.advertise<std_msgs::String>("Dest", 1000);
std_msgs::String msg3;
std::stringstream ss3;     //ss2,ss3,ss4;
ss3<<obj;
msg3.data =ss3.str();
ROS_INFO("%s", msg3.data.c_str());
chatter_Status.publish(msg3);



*/
cv::imshow(OPENCV_WINDOW, img);

//cv::imshow(OPENCV_WINDOW_1, imgThresholded);
cv::waitKey(3);
 // }
	
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
