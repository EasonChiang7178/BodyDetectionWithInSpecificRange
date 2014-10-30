#include <iostream>

//#include "K4Wv2ToOpenCV\K4Wv2ToOpenCV.h"
#include "BodyDetection\BodyDetection.h"

using namespace Kinect2;
using namespace std;

int main2(int argc, char** argv) {
	cv::setUseOptimized(true);

	K4Wv2ToOpenCV& theKinect2 = K4Wv2ToOpenCV::getDefaultKinectSensor();
	//theKinect2.setTryToReadingDataTimes(2000);
	//theKinect2.setTryToReadingDataInterval(1);

	theKinect2.initializeColorStream();
	theKinect2.initializeBodyStream();
	//theKinect2.initializeDepthStream();
	//theKinect2.initializeInfraredStream();
	//theKinect2.initializeBodyIndexStream();

	do {
		theKinect2.updataStreamData();
		theKinect2.drawBodySkeletonInColorImage();
		//theKinect2.drawBodySkeletonInDepthImage();

		const cv::Mat testColor = theKinect2.getColorImage();
		cv::Mat testColor2(cv::Size(testColor.cols / 2, testColor.rows / 2), CV_8UC4);
		cv::resize(testColor, testColor2, cv::Size(), 0.5, 0.5);

		//const cv::Mat testDepth = theKinect2.getDepthVisualizedImage();

		//const cv::Mat testInfrared = theKinect2.getInfraredVisulizedImage();

		//const cv::Mat testBodyIndex = theKinect2.getBodyIndexVisualizedImage();

		vector< Kinect2::Body > bodies = theKinect2.getBodyFrame().getBodies();
		for (auto bodiesIter = bodies.begin(); bodiesIter != bodies.end(); bodiesIter++) {
			auto joints = bodiesIter->getJointMap();

			if (bodiesIter->isTracked() == true) {
				cout << "> User ID: " << (int)bodiesIter->getIndex() << endl
					 << "  Confidence: " << bodiesIter->calcConfidence() << endl
					 << "  TrackedTime: " << (long long)bodiesIter->getTrackedTime() << endl
					 << "  SpineMid, x:" << joints[JointType_SpineMid].getPosition()[0] << endl
					 << "            y:" << joints[JointType_SpineMid].getPosition()[1] << endl
					 << "            z:" << joints[JointType_SpineMid].getPosition()[2] << endl << endl;
			}
		}

		cv::imshow("TEST COLOR", testColor2);
		//cv::imshow("TEST DEPTH", testDepth);
		//cv::imshow("TEST INFRARED", testInfrared);
		//cv::imshow("TEST BODY INDEX", testBodyIndex);
	} while (cv::waitKey(1) != 'q');

	theKinect2.closeKinectSensor();

	return 0;
}