#include <iostream>

#include "K4Wv2ToOpenCV\K4Wv2ToOpenCV.h"

using namespace Kinect2;
using namespace std;

int main(int argc, char** argv) {
	K4Wv2ToOpenCV& theKinect2 = K4Wv2ToOpenCV::GetDefaultKinectSensor();
	theKinect2.setTryToReadingDataTimes(2000);
	theKinect2.setTryToReadingDataInterval(1);

	theKinect2.initializeColorStream();

	do {
		theKinect2.updataStreamData();

		const cv::Mat test = theKinect2.getColorImage();
		cv::Mat test2(cv::Size(test.cols / 2, test.rows / 2), CV_8UC4);
		cv::resize(test, test2, cv::Size(), 0.5, 0.5);

		cv::imshow("TEST", test);
	} while (cv::waitKey(5) != 'q');

	theKinect2.closeKinectSensor();

	return 0;
}