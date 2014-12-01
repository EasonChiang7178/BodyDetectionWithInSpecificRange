#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "BodyDetection\BodyDetection.h"

#define COMMUICATION

using namespace Kinect2;
using namespace std;

string pathToConfig = "BodyDetection.cfg";
bool connectToServer = true;

int main(int argc, char** argv) {
	cv::setUseOptimized(true);

	cout << "\n\t\t< BodyDetection >" << endl;

	/* Loading the configuration */
	fstream fConfig;

	fConfig.open(pathToConfig.c_str(), ios::in);
	if (fConfig.is_open() == false) {
		cerr << "> [ERROR]: " << pathToConfig << " doesn't exist!" << endl;
		exit(EXIT_FAILURE);
	}
	
	vector< string > IPAddresses;
	vector< unsigned int > Ports;
	float upperRightX, upperRightY, upperLeftX, upperLeftY,
		  lowerRightX, lowerRightY, lowerLeftX, lowerLeftY;

		// Load each line...
	string lineEntry, entryName, entryValue;
	while (fConfig.eof() == false) {
		fConfig >> lineEntry;
		size_t colonPos = lineEntry.find_last_of(':');

		entryName = lineEntry.substr(0, colonPos);
		entryValue = lineEntry.substr(colonPos + 1);

		stringstream ss;

		if (entryName == "Address") {
			IPAddresses.push_back(entryValue);
		} else if (entryName == "Port") {
			unsigned int portBuffer;
			ss << entryValue; ss >> portBuffer;
			ss.str(""); ss.clear();
			Ports.push_back(portBuffer);

		} else if (entryName == "UpperLeftCorner_X") {
			ss << entryValue; ss >> upperLeftX;
		} else if (entryName == "UpperLeftCorner_Y") {
			ss << entryValue; ss >> upperLeftY;
		} else if (entryName == "UpperRightCorner_X") {
			ss << entryValue; ss >> upperRightX;
		} else if (entryName == "UpperRightCorner_Y") {
			ss << entryValue; ss >> upperRightY;
		} else if (entryName == "LowerLeftCorner_X") {
			ss << entryValue; ss >> lowerLeftX;
		} else if (entryName == "LowerLeftCorner_Y") {
			ss << entryValue; ss >> lowerLeftY;
		} else if (entryName == "LowerRightCorner_X") {
			ss << entryValue; ss >> lowerRightX;
		} else if (entryName == "LowerRightCorner_Y") {
			ss << entryValue; ss >> lowerRightY;
		}
	}

	fConfig.close();

	/* Starting Body Detection */
	BodyDetection& bodyDetector = BodyDetection::getDefaultKinectSensor();

	bodyDetector.setRegion(cv::Vec2f(upperLeftX, upperLeftY), cv::Vec2f(upperRightX, upperRightY),
						   cv::Vec2f(lowerRightX, lowerRightY), cv::Vec2f(lowerLeftX, lowerLeftY));

	bodyDetector.initializeColorStream();
	bodyDetector.initializeBodyStream();

		// Connect to the servers
#ifdef COMMUICATION
	for (size_t serverIndex = 0; serverIndex < IPAddresses.size(); serverIndex++) {
		try {
			bodyDetector.connectTo(Ports[serverIndex], IPAddresses[serverIndex]);
			cout << "> [INFO] Connect to " << IPAddresses[serverIndex] << ":" << Ports[serverIndex] << " successfully!" << endl;
		}
		catch (exception& e) {
			cerr << "> [ERROR] " << e.what() << " in " << IPAddresses[serverIndex] << ":" << Ports[serverIndex] << endl
				<< "                   (Executing offline...)" << endl
				<< "                   (Please press enter to continue)" << endl;
			connectToServer = false;

			char pause;
			pause = getchar();
		}
	}
#endif // COMMUICATION

	cout << "\n> [INFO] Please press ''q'' or ''Esc'' to exit" << endl;

		// Declare the variables for calculating the fps
	float fps = 0.0f;
	unsigned int frameCounter = 0;
	int64 startTime = cv::getTickCount() , currentTime = 0L;

	/* Main Loop */
	char checkExit = '\0';
	do {
		bodyDetector.updataStreamData();
		
		bodyDetector.drawBodiesInColorImage();
		
		bodyDetector.checkAllBodiesInRegion();
		bodyDetector.drawRegionInColorImage();
		bodyDetector.drawBodiesInRegion();
		bodyDetector.drawUserPosition();

		if (connectToServer == true)
			bodyDetector.sendMessage();

#ifdef _DEBUG
		vector< bodyDetectedMessage > messagesSent = bodyDetector.getMessagesToSend();
		if (messagesSent.size() != 0) {

			for (auto messageIter = messagesSent.begin(); messageIter != messagesSent.end(); messageIter++)
				std::cout << "> [Message Sent]: " << messageIter->getMessage() << endl;
			
			std::cout << endl;

			std::cout << "\t\t[User Information]" << endl;
			vector< cv::Vec3f > userPosition = bodyDetector.getUsersPosition();
			vector< bool > userInRegion = bodyDetector.isUsersInRegion();
			vector< Kinect2::Body > bodies = bodyDetector.getBodyFrame().getBodies();
			for (auto bodiesIter = bodies.begin(); bodiesIter != bodies.end(); bodiesIter++) {
				auto joints = bodiesIter->getJointMap();

				if (bodiesIter->isTracked() == true) {
					std::cout << "> Index:       " << static_cast< unsigned short>(bodiesIter->getIndex()) << endl
							  << "  User ID:     " << bodiesIter->getId() << endl
							  << "  Confidence:  " << bodiesIter->calcConfidence() << endl
							  << "  TrackedTime: " << (long long)bodiesIter->getTrackedTime() << endl
							  << "  InRegion:    " << userInRegion[bodiesIter->getIndex()] << endl
							  << "  Centor,   x: " << userPosition[bodiesIter->getIndex()][0] << endl
							  << "            y: " << userPosition[bodiesIter->getIndex()][1] << endl
							  << "            z: " << userPosition[bodiesIter->getIndex()][2] << endl << endl
							  << "  Head,     x: " << joints[JointType_Head].getPosition()[0] << endl
							  << "            y: " << joints[JointType_Head].getPosition()[1] << endl
							  << "            z: " << joints[JointType_Head].getPosition()[2] << endl << endl;
				}
			}
			std::cout << "============== Info ==============" << endl;

			std::cout << endl;
		}
#endif
			// Prepare for display the image
		const cv::Mat rawColorImg = bodyDetector.getColorImage();
		cv::Mat bodyDetectionColorImg(cv::Size(rawColorImg.cols / 2, rawColorImg.rows / 2), CV_8UC4);
		cv::resize(rawColorImg, bodyDetectionColorImg, cv::Size(), 0.5, 0.5);

		/* Computing the current fps */
		currentTime = cv::getTickCount();
		fps = static_cast< float >(++frameCounter / ((currentTime - startTime) / cv::getTickFrequency()));

		std::ostringstream os;
		os << fps;
		std::string fpsStr = os.str();
		fpsStr = "FPS: " + fpsStr;

		//cout << "FPS: " << fps << endl;
		cv::putText(bodyDetectionColorImg, fpsStr, cv::Point(0, 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 200), 2, CV_AA);

		cv::imshow("Body Detction", bodyDetectionColorImg);
		checkExit = cv::waitKey(1);
	} while (checkExit != 27 && checkExit != 'q' && checkExit != 'Q');

	//bodyDetector.closeKinectSensor();
	return 0;
}