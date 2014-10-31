#include <iostream>

//#include "K4Wv2ToOpenCV\K4Wv2ToOpenCV.h"
#include "BodyDetection\BodyDetection.h"

using namespace Kinect2;
using namespace std;

int main(int argc, char** argv) {
	cv::setUseOptimized(true);

	//K4Wv2ToOpenCV& theKinect2 = K4Wv2ToOpenCV::getDefaultKinectSensor();
	BodyDetection& theKinect2 = BodyDetection::getDefaultKinectSensor();

	theKinect2.initializeColorStream();
	theKinect2.initializeBodyStream();

	do {
		theKinect2.updataStreamData();
		theKinect2.drawRegionInColorImage();
		theKinect2.drawBodiesInColorImage();
		
		theKinect2.checkAllBodiesInRegion();
		theKinect2.drawBodiesInRegion();
		theKinect2.drawUserPosition();
		//theKinect2.drawUserPositionInRegion();

		const cv::Mat testColor = theKinect2.getColorImage();
		cv::Mat testColor2(cv::Size(testColor.cols / 2, testColor.rows / 2), CV_8UC4);
		cv::resize(testColor, testColor2, cv::Size(), 0.5, 0.5);

		//const cv::Mat testDepth = theKinect2.getDepthVisualizedImage();

		//const cv::Mat testInfrared = theKinect2.getInfraredVisulizedImage();

		//const cv::Mat testBodyIndex = theKinect2.getBodyIndexVisualizedImage();

		vector< cv::Vec3f > userPosition = theKinect2.getUsersPosition();
		vector< bool > userInRegion = theKinect2.isUsersInRegion();
		vector< Kinect2::Body > bodies = theKinect2.getBodyFrame().getBodies();
		for (auto bodiesIter = bodies.begin(); bodiesIter != bodies.end(); bodiesIter++) {
			auto joints = bodiesIter->getJointMap();

			if (bodiesIter->isTracked() == true) {
				cout << "> User ID:     " << (int)bodiesIter->getId() << endl
					 << "  Confidence:  " << bodiesIter->calcConfidence() << endl
					 << "  TrackedTime: " << (long long)bodiesIter->getTrackedTime() << endl
					 << "  InRegion:    " << userInRegion[bodiesIter->getIndex()] << endl
					 << "  Position, x: " << userPosition[bodiesIter->getIndex()][0] << endl
					 << "            y: " << userPosition[bodiesIter->getIndex()][1] << endl
					 << "            z: " << userPosition[bodiesIter->getIndex()][2] << endl << endl;
					 //<< "  Position, x: " << joints[JointType_HandRight].getPosition()[0] << endl
					 //<< "            y: " << joints[JointType_HandRight].getPosition()[1] << endl
					 //<< "            z: " << joints[JointType_HandRight].getPosition()[2] << endl << endl;
			}
		}

		cv::imshow("TEST COLOR", testColor2);
	} while (cv::waitKey(1) != 'q');

	theKinect2.closeKinectSensor();

	return 0;
}