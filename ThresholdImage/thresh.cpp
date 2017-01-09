#include "thresh.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

Mat thresh::redFilter(const Mat& src)
{

    Mat redOnly;
    Mat convert = colorCon(src);

    //cv::inRange(convert, cv::Scalar(52, 14, 154), cv::Scalar(70, 50, 220), redOnly);

    //cv::inRange(convert, cv::Scalar(thAry[0] - 50, thAry[1] - 50, thAry[2] - 50), cv::Scalar(thAry[0] + 30, thAry[1] + 30, thAry[2] + 30), redOnly);
    cv::inRange(convert, cv::Scalar(10, 10, 178), cv::Scalar(180, 150, 255), redOnly);
    //threshold( convert, redOnly, threshold_value, max_BINARY_value,threshold_type );
    //Red threthold
    //cv::inRange(convert, cv::Scalar(115, 100, 100), cv::Scalar(155, 255, 255), redOnly);

    return redOnly;
}
Mat thresh::blueFilter(const Mat& src)
{

    Mat redOnly;
    Mat convert = colorCon(src);
    cv::inRange(convert, cv::Scalar(52, 14, 154), cv::Scalar(70, 50, 220), redOnly);
    return redOnly;
}



Mat thresh::colorCon(const Mat& src)
{
	Mat hsv;

	cvtColor(src,hsv,CV_RGB2HSV);

	return hsv;
}

void thresh::findHSVThresh(){
	int lowH, highH, lowS, highS, lowV, highV;
	VideoCapture vcap(0);
	Mat con;
	Mat filtColor;

	while(true){
		vcap.read(con);
		colorCon(con);
		namedWindow("Control", CV_WINDOW_AUTOSIZE);
		cvCreateTrackbar("Low Hue", "Control", &lowH, 255);
		cvCreateTrackbar("High Hue", "Control", &highH, 255);
		cvCreateTrackbar("Low Saturation", "Control", &lowS, 255);
		cvCreateTrackbar("High Saturation", "Control", &highS, 255);
		cvCreateTrackbar("Low Value", "Control", &lowV, 255);
		cvCreateTrackbar("High Value", "Control", &highV, 255);
		cv::inRange(con, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), filtColor);

		imshow("camera view", con);
		imshow("filtered view", filtColor);

		if (waitKey(10) >= 0){
			break;
			vcap.release();

		}

	}
}
void thresh::findRGBThresh(){
	int lowH, highH, lowS, highS, lowV, highV;
	VideoCapture vcap(0);
	Mat con;
	Mat filtColor;

	while(true){
		vcap.read(con);

		namedWindow("Control", CV_WINDOW_AUTOSIZE);
		cvCreateTrackbar("Low Hue", "Control", &lowH, 0);
		cvCreateTrackbar("High Hue", "Control", &highH, 255);
		cvCreateTrackbar("Low Saturation", "Control", &lowS, 0);
		cvCreateTrackbar("High Saturation", "Control", &highS, 255);
		cvCreateTrackbar("Low Value", "Control", &lowV, 0);
		cvCreateTrackbar("High Value", "Control", &highV, 255);
		cv::inRange(con, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), filtColor);

		imshow("camera view", con);
		imshow("filtered view", filtColor);

		if (waitKey(10) >= 0){
			break;
			vcap.release();

		}

	}
}

//Point p;

void thresh::callBack(int event, int x, int y, int flags, void* userdata){
	if(event == CV_EVENT_LBUTTONDOWN){
		Mat frame;
		//Mat image=frame.clone();
		Vec3b rgb=frame.at<Vec3b>(y,x);
		int B=rgb.val[0];
		int G=rgb.val[1];
		int R=rgb.val[2];
		Point p(x, y);
		char name[30];

		thresh thrIm;
		Mat im = thrIm.colorCon(frame);
		Vec3b hsv=im.at<Vec3b>(y,x);
		int H=hsv.val[0];
		int S=hsv.val[1];
		int V=hsv.val[2];
		Point hp(x, y);
		char name1[30];

		sprintf(name,"R=%d",R);
		putText(frame,name, Point(20,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name,"G=%d",G);
		putText(frame,name, Point(20,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name,"B=%d",B);
		putText(frame,name, Point(20,120), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		sprintf(name1,"H = %d",H);
		putText(frame,name1, Point(100,40), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name1,"S = %d",S);
		putText(frame,name1, Point(100,80), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );
		sprintf(name1,"V = %d",V);
		putText(frame,name1, Point(100,120), FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );



		sprintf(name,"X=%d",x);
		putText(frame,name, Point(20,200) , FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		sprintf(name,"Y=%d",y);
		putText(frame,name, Point(20,240) , FONT_HERSHEY_SIMPLEX, .7, Scalar(255,0,0), 2,8,false );

		 //imwrite("hsv.jpg",image);
		 imshow("Image Threshold", frame );
	}

}

void thresh::clickThreshold(){
	//void* data = 0;
	VideoCapture vcap(0);
	Mat frame;

	if(!vcap.isOpened()){
		cout  << "Video capture is not opened!" << endl;
	}
	void* userdata = 0;

	while(true){
		vcap.read(frame);
		namedWindow("Window", CV_WINDOW_AUTOSIZE);
		cv::imshow("Window", frame);
		cv::setMouseCallback("Window", callBack, userdata);

		if (waitKey(10) >= 0){
			break;
		}

	}

}
