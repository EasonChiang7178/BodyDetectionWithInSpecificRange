#include <iostream>

#include "K4Wv2ToOpenCV\K4Wv2ToOpenCV.h"

using namespace Kinect2;
using namespace std;

int main(int argc, char** argv) {
	cv::setUseOptimized(true);

	K4Wv2ToOpenCV& theKinect2 = K4Wv2ToOpenCV::GetDefaultKinectSensor();
	theKinect2.setTryToReadingDataTimes(2000);
	theKinect2.setTryToReadingDataInterval(1);

	theKinect2.initializeColorStream();
	theKinect2.initializeDepthStream();
	theKinect2.initializeInfraredStream();
	theKinect2.initializeBodyIndexStream();

	do {
		theKinect2.updataStreamData();

		const cv::Mat testColor = theKinect2.getColorImage();
		cv::Mat testColor2(cv::Size(testColor.cols / 2, testColor.rows / 2), CV_8UC4);
		cv::resize(testColor, testColor2, cv::Size(), 0.5, 0.5);

		const cv::Mat testDepth = theKinect2.getDepthVisualizedImage();

		const cv::Mat testInfrared = theKinect2.getInfraredVisulizedImage();

		const cv::Mat testBodyIndex = theKinect2.getBodyIndexVisualizedImage();

		cv::imshow("TEST COLOR", testColor2);
		cv::imshow("TEST DEPTH", testDepth);
		cv::imshow("TEST INFRARED", testInfrared);
		cv::imshow("TEST BODY INDEX", testBodyIndex);
	} while (cv::waitKey(1) != 'q');

	theKinect2.closeKinectSensor();

	return 0;
}