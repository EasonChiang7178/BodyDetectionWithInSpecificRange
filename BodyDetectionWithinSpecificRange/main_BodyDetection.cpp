#include <iostream>

#include "BodyDetection\BodyDetection.h"

using namespace Kinect2;
using namespace std;

int main(int argc, char** argv) {
	cv::setUseOptimized(true);

	BodyDetection& bodyDetector = BodyDetection::getDefaultKinectSensor();

	bodyDetector.initializeColorStream();
	bodyDetector.initializeBodyStream();

	bodyDetector.connectTo(1234, "192.168.1.132");

	do {
		bodyDetector.updataStreamData();
		bodyDetector.drawRegionInColorImage();
		bodyDetector.drawBodiesInColorImage();
		
		bodyDetector.checkAllBodiesInRegion();
		bodyDetector.drawBodiesInRegion();
		bodyDetector.drawUserPosition();
		
		const cv::Mat testColor = bodyDetector.getColorImage();
		cv::Mat testColor2(cv::Size(testColor.cols / 2, testColor.rows / 2), CV_8UC4);
		cv::resize(testColor, testColor2, cv::Size(), 0.5, 0.5);

		bodyDetector.sendMessage();

		vector< bodyDetectedMessage > messagesSent = bodyDetector.getMessagesToSend();
		for (auto messageIter = messagesSent.begin(); messageIter != messagesSent.end(); messageIter++) {
				cout << "> User ID:     " << messageIter->userID << endl
					 << "  isInRegion:  " << messageIter->detectedBody << endl << endl;
		}

		//vector< cv::Vec3f > userPosition = bodyDetector.getUsersPosition();
		//vector< bool > userInRegion = bodyDetector.isUsersInRegion();
		//vector< Kinect2::Body > bodies = bodyDetector.getBodyFrame().getBodies();
		//for (auto bodiesIter = bodies.begin(); bodiesIter != bodies.end(); bodiesIter++) {
		//	auto joints = bodiesIter->getJointMap();

		//	if (bodiesIter->isTracked() == true) {
		//		cout << "> User ID:     " << (int)bodiesIter->getId() << endl
		//			 << "  Confidence:  " << bodiesIter->calcConfidence() << endl
		//			 << "  TrackedTime: " << (long long)bodiesIter->getTrackedTime() << endl
		//			 << "  InRegion:    " << userInRegion[bodiesIter->getIndex()] << endl
		//			 << "  Position, x: " << userPosition[bodiesIter->getIndex()][0] << endl
		//			 << "            y: " << userPosition[bodiesIter->getIndex()][1] << endl
		//			 << "            z: " << userPosition[bodiesIter->getIndex()][2] << endl << endl;
		//			 //<< "  Position, x: " << joints[JointType_HandRight].getPosition()[0] << endl
		//			 //<< "            y: " << joints[JointType_HandRight].getPosition()[1] << endl
		//			 //<< "            z: " << joints[JointType_HandRight].getPosition()[2] << endl << endl;
		//	}
		//}

		cv::imshow("TEST COLOR", testColor2);
	} while (cv::waitKey(1) != 'q');

	bodyDetector.closeKinectSensor();

	return 0;
}