#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>
#include <stdio.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include "imageProcessing.h"
#include "Xmlio.h"
#include "thresh.h"

using namespace std;
using boost::property_tree::ptree;
using boost::property_tree::write_xml;
using boost::property_tree::xml_writer_settings;

//VideoCapture vcap(0);
thresh* thsh = new thresh();
imageProcessing det;
Xmlio* mlio = new Xmlio;
std::MouseCallback* callbk = new MouseCallback;
std::ofstream outfile ("SavedThresholds.txt");
string xthresh[6];
Mat cameraFrame;

ros::CvImage cameraFrame_br;
ros::Image img_msg;
cameraFrame_br = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cameraFrame);
cameraFrame_br.toImageMsg(img_msg);
pub_img.publish(img_msg);

int thresholdValue = 100;
int max_thresh = 255;
int thAry[6];

std::vector<std::string> topics;

bool rclick = false;

/**
 * Left button call back that finds the location of the mouse pointer and the rgb and hsv values at that point.
 * Creates a Mat and displays the rgb, hsv, and xy values on the Mat
 *
 * @param int stores the type of mouse event(left button click, mouse scroll)
 * @param int stores the x-axis location of the mouse pointer
 * @param int stores the y-axis location of the mouse pointer
 * @param int does nothing
 * @param void* I HAVE NO IDEA
 */
void imageProcessing::rCall_Back(int event, int x, int y, int flags, void* userdata){
	if(event == CV_EVENT_LBUTTONDOWN){

		Mat image, retIm;
		int thArray[6];

		vcap.release();
		vcap.open(0);
		vcap.read(image);

		Vec3b rgb=image.at<Vec3b>(y,x);
		int B = rgb.val[0];
		int G = rgb.val[1];
		int R = rgb.val[2];
		Point p(x, y);
		char name[30];

		Mat im = thsh -> colorCon(image);
		Vec3b hsv=im.at<Vec3b>(y,x);
		int H = hsv.val[0];
		int S = hsv.val[1];
		int V = hsv.val[2];
		char name1[30];

		sprintf(name,"R=%d",R);
		putText(image,name, Point(20,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name,"G=%d",G);
		putText(image,name, Point(20,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name,"B=%d",B);
		putText(image,name, Point(20,120), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		sprintf(name1,"H = %d",H);
		putText(image,name1, Point(100,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name1,"S = %d",S);
		putText(image,name1, Point(100,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name1,"V = %d",V);
		putText(image,name1, Point(100,120), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );



		sprintf(name,"X=%d",x);
		putText(image,name, Point(20,200) , FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		sprintf(name,"Y=%d",y);
		putText(image,name, Point(20,240) , FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		thArray[0] = B;
		thArray[1] = G;
		thArray[2] = R;
		thArray[3] = H;
		thArray[4] = S;
		thArray[5] = V;

		mlio -> writeXml(thArray);
		mlio -> writeText(thArray);

		imshow("Image Threshold", image );

	    cv::inRange(im, cv::Scalar(H - 20, S - 20, V - 20), cv::Scalar(H + 20, S + 20, V + 20), retIm);
		imshow("threshold result", retIm);

		cv::moveWindow("Image Threshold", 1100, 10);
		cv::moveWindow("threshold result", 650, 600);
	}
	else if(event == CV_EVENT_RBUTTONDOWN){
		rclick = true;
	}
}

/**
 * reads the video capture and creates a new mouse call back.
 */
void imageProcessing::clickThresh(){
	void* userdata = 0;
	Mat src;
	Mat frame;
	//vcap.release();
	//vcap.open(0);

	//if(!vcap.isOpened()){
	//		cout  << "Video capture is not opened!" << endl;
	//	}

	while(true){

		//vcap.read(frame);

		namedWindow("Window", CV_WINDOW_AUTOSIZE);
		cv::imshow("Window", frame);
		cv::moveWindow("Window", 200, 10);
		cv::setMouseCallback("Window", rCall_Back, userdata);
		if (rclick || waitKey(20) >= 0){
			cvDestroyWindow("threshold result");
			cvDestroyWindow("Image Threshold");
			break;
		}
	}
}

/**
 * call back used to switch to a thresholding view
 *
 * @param int stores the type of mouse event(left button click, mouse scroll)
 * @param int stores the x-axis location of the mouse pointer
 * @param int stores the y-axis location of the mouse pointer
 * @param int does nothing
 * @param void* I HAVE NO IDEA
 */
void imageProcessing::call_Back(int event, int x, int y, int flags, void* userdata){
	if(event == CV_EVENT_RBUTTONDOWN){
		cvDestroyWindow("HSV");
		cvDestroyWindow("Filtered Color");

		det.clickThresh();
	}
}

void imageProcessing::call_Back2(int event, int x, int y, int flags, void* userdata){
	if(event == CV_EVENT_RBUTTONDOWN){
		cvDestroyWindow("HSV");
		cvDestroyWindow("Filtered Color");
		//vcap.release();
		thsh -> findHSVThresh();
	}
}

int main(int argc, char** argv){

	void* data = 0;
	Mat camera2, hsv, red_Only, blue_Only, filt;
	ptree pt;
	read_xml("Thresholds.xml", pt);
	string xthresh;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

    ros::init(argc, argv, "image_publisher");
	ros::NodeHandle handle;
	ros::Subscriber sub = handle.subscribe("image", 1, image_input);

	//namedWindow("Window", CV_WINDOW_AUTOSIZE);
	//namedWindow("Filtered Color", CV_WINDOW_AUTOSIZE);

	cv::SimpleBlobDetector::Params params;
	params.filterByArea = true;
	params.minArea = 100.0;
	params.maxArea = 50000.0;

	while(true){

		//vcap.read(cameraFrame);

		rclick = false;

		hsv = thsh -> colorCon(cameraFrame);
		red_Only = thsh -> redFilter(hsv);
		red_Only = thsh -> redFilter(hsv);
		cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		morphologyEx(red_Only, cameraFrame, cv::MORPH_OPEN, str_el);

		//cameraFrame = findContours(red_Only, RETR_TREE, CHAIN_APPROX_SIMPLE);
		//Canny( red_Only, red_Only, thresholdValue, thresholdValue*2, 3 );
		findContours( cameraFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		Mat drawing = Mat::zeros( cameraFrame.size(), CV_8UC3 );
		  for( int i = 0; i< contours.size(); i++ )
		     {
		       //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		     }
		cv::Mat matF;
		matF.push_back(red_Only);
		matF.push_back(blue_Only);
		//auto channels = std::vector<cv::Mat>{red_Only, blue_Only};
		//cv::merge(channels, filt);
		imshow("HSV", hsv);
		imshow("Filtered Color", matF);
		imshow("Window", cameraFrame);

		cv::moveWindow("HSV", 1100, 10);
		cv::moveWindow("Filtered Color", 650, 600);
		cv::moveWindow("Window", 200, 10);

		cv::setMouseCallback("Window", det.call_Back , data);
		cv::setMouseCallback("Filtered Color", det.call_Back2 , data);
		if (waitKey(10) >= 0){
			break;
		}

	}
	return 0;
}

