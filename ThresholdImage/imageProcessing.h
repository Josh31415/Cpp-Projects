/*
 * imageProcessing.h
 *
 *  Created on: Oct 27, 2016
 *      Author: josh
 */

#ifndef SRC_IMAGEPROCESSING_H_
#define SRC_IMAGEPROCESSING_H_

namespace std {

class imageProcessing {
public:
	cv::Mat color_Con(const cv::Mat& src);
	cv::Mat red_Filter(const cv::Mat& src);
	void clickThresh();
	static std::vector<int> readml();
	static void rCall_Back(int event, int x, int y, int flags, void* userdata);
	static void call_Back(int event, int x, int y, int flags, void* userdata);
	static void call_Back2(int event, int x, int y, int flags, void* userdata);
};

} /* namespace std */

#endif /* SRC_IMAGEPROCESSING_H_ */
