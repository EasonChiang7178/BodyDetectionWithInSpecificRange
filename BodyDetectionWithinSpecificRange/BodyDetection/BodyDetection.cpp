#include "BodyDetection.h"

/** Constructors **/
BodyDetection& BodyDetection::getDefaultKinectSensor() {
	static BodyDetection instance;

	return instance;
}

BodyDetection::BodyDetection()
	:K4Wv2ToOpenCV()
{
	userPosition.resize(BODY_COUNT);
	userID.resize(BODY_COUNT);
	userInRegion.resize(BODY_COUNT);
	//userWavingHand.resize(BODY_COUNT);

	//this->streams.resize(0);
	messagesToSend.resize(0);

	this->setRegion();
}

void BodyDetection::checkAllBodiesInRegion() {
	for (unsigned char bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
		this->extractUserPositionWithUpperBodyAverage(bodyIndex);
		userInRegion[bodyIndex] = this->checkBodyInRectangularRegion(bodyIndex, userPosition[bodyIndex]);
	}
}


// Reference: http://kone.vis.ne.jp/diary/diaryb09.html
const bool BodyDetection::checkBodyInRectangularRegion(const int bodyIndex, const cv::Vec3f& userPosition) {
	if (this->userPosition[bodyIndex] == cv::Vec3f(0.0, 0.0, 0.0))
		return false;

	bool detectedResult = true;

	std::vector< cv::Vec2f > corners;
	corners.push_back(upperLeftCorner); corners.push_back(upperRightCorner);
	corners.push_back(lowerRightCorner); corners.push_back(lowerLeftCorner);
	
	float sumDeg = 0.0f;

	for (int i = 0; i < 4; i++) {
			//       x(= z)                z            y(= x)             x
		cv::Vec2f l1(corners[i][0] - userPosition[2], corners[i][1] - userPosition[0]);
		cv::Vec2f l2(corners[(i + 1) % 4][0] - userPosition[2], corners[(i + 1) % 4][1] - userPosition[0]);

		float cosValue = l1[0] * l2[0] + l1[1] * l2[1];
		float sinValue = l1[0] * l2[1] - l2[0] * l1[1];

		sumDeg += static_cast< float >(atan2(sinValue, cosValue) * 180.0f / PI);
	}

	int sumDeg_int = static_cast< int >((round(sumDeg)) / 360);
	if (sumDeg_int != 1 && sumDeg_int != -1)
		detectedResult = false;

	return detectedResult;
}

const std::vector< cv::Vec3f >&	BodyDetection::getUsersPosition() const {
	return userPosition;
}

const std::vector< unsigned long long >& BodyDetection::getUsersID() const {
	return userID;
}

const std::vector< bool >& BodyDetection::isUsersInRegion() const {
	return userInRegion;
}

const std::vector< bodyDetectedMessage >& BodyDetection::getMessagesToSend() const {
	return messagesToSend;
}

const bool BodyDetection::setRegion(cv::Vec2f upperLeft, cv::Vec2f upperRight,
									cv::Vec2f lowerRight, cv::Vec2f lowerLeft) {
	this->upperLeftCorner = upperLeft;
	this->upperRightCorner = upperRight;
	this->lowerLeftCorner = lowerLeft;
	this->lowerRightCorner = lowerRight;

	return true;
}

const bool BodyDetection::connectTo(const int& port, const std::string& serverAddress) {
	TCPStream stream = this->connector.connect(port, serverAddress);
	streams.push_back(stream);
	return true;
}

const bool BodyDetection::sendMessage() {
		// Feed the data into messagesToSend vector
	this->prepareMessageToSend();

	for (vector< TCPStream >::iterator streamsIter = streams.begin(); streamsIter != streams.end(); streamsIter++) {
		streamsIter->addMessages(messagesToSend);
		streamsIter->send();
	}
	return true;
}

const bool BodyDetection::drawRegionInColorImage(cv::Scalar regionColor) {
	prepareDrawRectangularRegion();

	cv::circle(cvColorMat, cv::Point(ulColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(urColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(llColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(lrColorPoint), 5, regionColor, CV_FILLED, CV_AA);

	cv::line(cvColorMat, cv::Point(ulColorPoint), cv::Point(urColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(ulColorPoint), cv::Point(llColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(urColorPoint), cv::Point(lrColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(llColorPoint), cv::Point(lrColorPoint), regionColor, 3, CV_AA);

	cv::circle(cvColorMat, cv::Point(ulColorPointCell), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(urColorPointCell), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(llColorPointCell), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(lrColorPointCell), 5, regionColor, CV_FILLED, CV_AA);

	cv::line(cvColorMat, cv::Point(ulColorPointCell), cv::Point(urColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(ulColorPointCell), cv::Point(llColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(urColorPointCell), cv::Point(lrColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(llColorPointCell), cv::Point(lrColorPointCell), regionColor, 3, CV_AA);

	cv::line(cvColorMat, cv::Point(ulColorPoint), cv::Point(ulColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(lrColorPoint), cv::Point(lrColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(urColorPoint), cv::Point(urColorPointCell), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(llColorPoint), cv::Point(llColorPointCell), regionColor, 3, CV_AA);

	return true;
}

const bool BodyDetection::drawBodiesInRegion(cv::Scalar bodyColor) {
	for (int bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
		if (userInRegion[bodyIndex] == true)
			this->drawBodyInColorImage(bodyIndex, bodyColor, bodyColor);
	}

	return true;
}

const bool BodyDetection::drawUserPosition() {
	for (int bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
		this->drawUserPoint(bodyIndex, static_cast< cv::Scalar >(this->colorizeBody(bodyIndex)));
	}

	return true;
}

const bool BodyDetection::drawUserPositionInRegion(cv::Scalar pointColor) {
	for (int bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
		if (userInRegion[bodyIndex] == true)
			this->drawUserPoint(bodyIndex, pointColor);
	}

	return true;
}

const bool BodyDetection::drawUserPoint(const int bodyIndex, cv::Scalar pointColor) {
	if (userPosition[bodyIndex] != cv::Vec3f(0.0, 0.0, 0.0)) {
		cv::Point p(this->mapCameraToColor(userPosition[bodyIndex]));

		cv::circle(cvColorMat, p, 10, pointColor, 10);

		return true;
	}

	return false;
}

void BodyDetection::prepareDrawRectangularRegion(const float yAxisUpperOffset, const float yAxisLowerOffset) {
	cv::Vec3f upperLeftCorner3d(upperLeftCorner[1], yAxisUpperOffset, upperLeftCorner[0]),
			  upperRightCorner3d(upperRightCorner[1], yAxisUpperOffset, upperRightCorner[0]),
			  lowerLeftCorner3d(lowerLeftCorner[1], yAxisLowerOffset, lowerLeftCorner[0]),
			  lowerRightCorner3d(lowerRightCorner[1], yAxisLowerOffset, lowerRightCorner[0]),
			  upperLeftCorner3dCell(upperLeftCorner[1], yAxisUpperOffset + 0.8f, upperLeftCorner[0]),
			  upperRightCorner3dCell(upperRightCorner[1], yAxisUpperOffset + 0.8f, upperRightCorner[0]),
			  lowerLeftCorner3dCell(lowerLeftCorner[1], yAxisLowerOffset + 0.8f, lowerLeftCorner[0]),
			  lowerRightCorner3dCell(lowerRightCorner[1], yAxisLowerOffset + 0.8f, lowerRightCorner[0]);

	ulColorPoint = this->mapCameraToColor(upperLeftCorner3d);
	urColorPoint = this->mapCameraToColor(upperRightCorner3d);
	llColorPoint = this->mapCameraToColor(lowerLeftCorner3d);
	lrColorPoint = this->mapCameraToColor(lowerRightCorner3d);
	ulColorPointCell = this->mapCameraToColor(upperLeftCorner3dCell);
	urColorPointCell = this->mapCameraToColor(upperRightCorner3dCell);
	llColorPointCell = this->mapCameraToColor(lowerLeftCorner3dCell);
	lrColorPointCell = this->mapCameraToColor(lowerRightCorner3dCell);
}

void BodyDetection::prepareMessageToSend() {
	messagesToSend.clear();
	messagesToSend.resize(0);

	for (int index = 0; index < BODY_COUNT; index++) {
		if (cvBodyFrame.getBodies()[index].isTracked() == false)
			continue;
		
		bodyDetectedMessage tempMessage;
		tempMessage.userID = this->userID[index];
		tempMessage.detectedBody = this->userInRegion[index];

		messagesToSend.push_back(tempMessage);
	}
}

const cv::Vec3f BodyDetection::extractUserPositionWithUpperBodyAverage(const int bodyIndex) {
	cv::Vec3f headPoint = this->getUserSkeletonPosition(bodyIndex, JointType_Head);
	cv::Vec3f spineShoulderPoint = this->getUserSkeletonPosition(bodyIndex, JointType_SpineShoulder);
	cv::Vec3f spineMidPoint = this->getUserSkeletonPosition(bodyIndex, JointType_SpineMid);
		// x direction
	userPosition[bodyIndex][0] = (headPoint[0] + spineShoulderPoint[0] + spineMidPoint[0]) / 3.0f;
		// y direction
	userPosition[bodyIndex][1] = (headPoint[1] + spineShoulderPoint[1] + spineMidPoint[1]) / 3.0f;
		// z direction
	userPosition[bodyIndex][2] = (headPoint[2] + spineShoulderPoint[2] + spineMidPoint[2]) / 3.0f;

	return userPosition[bodyIndex];
}

const cv::Vec3f BodyDetection::getUserSkeletonPosition(const int bodyIndex, const JointType jointName) {
	auto userJoints = cvBodyFrame.getBodies()[bodyIndex].getJointMap();
	return userJoints[jointName].getPosition();
}

void bodyDetectedMessage::makeMessage() {
	std::string space = " ";

	std::stringstream ss;
	std::string userIDstr;
	ss << this->userID; ss >> userIDstr;

	std::string userInRegionFlag = "false";
	if (this->detectedBody = true)
		userInRegionFlag = "true";
	this->messageToSend = userIDstr + space + userInRegionFlag;
}