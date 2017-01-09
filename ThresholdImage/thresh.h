/*
 * thresh.h
 *
 *  Created on: Oct 27, 2016
 *      Author: josh
 */

#ifndef SRC_THRESH_H_
#define SRC_THRESH_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>
#include <stdio.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/opencv.hpp>

namespace std {

class thresh {

public:
	cv::Mat redFilter(const cv::Mat& src);
	cv::Mat blueFilter(const cv::Mat& src);
	cv::Mat colorCon(const cv::Mat& src);
	void findRGBThresh();
	void findHSVThresh();
	void clickThreshold();
	static void callBack(int event, int x, int y, int flags, void* userdata);
	static void rCallBack(int event, int x, int y, int flags, void* userdata);

};

} /* namespace std */

#endif /* SRC_THRESH_H_ */
