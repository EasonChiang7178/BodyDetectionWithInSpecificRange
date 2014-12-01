#ifndef BODY_DETECTION_REGION_H
#define BODY_DETECTION_REGION_H

#define PI 3.14159265

#include <sstream>
#include <string>
#include <cmath>

#include "K4Wv2ToOpenCV/K4Wv2ToOpenCV.h"
#include "TCPClient/TCPConnector.h"

using namespace Kinect2;

class bodyDetectedMessage : public message {
public:
	float x;
	float y;
	float z;

	bool detectedBody;
	unsigned long long userID;

protected:
	virtual void makeMessage();
};

class BodyDetection : public K4Wv2ToOpenCV {
public:
		// Constructor with Singleton Pattern
	static BodyDetection& getDefaultKinectSensor();
	
	void checkAllBodiesInRegion();

	/* Accessor Set */
	const std::vector< cv::Vec3f >&				getUsersPosition() const;
	const std::vector< unsigned long long >&	getUsersID() const;
	const std::vector< bool >&					isUsersInRegion() const;
	//const std::vector< bool >&				isUsersWavingHand() const;

	const std::vector< bodyDetectedMessage >&	getMessagesToSend() const;

	/* Configuration Set */
	const bool setRegion(cv::Vec2f upperLeft = cv::Vec2f(2.0, 0.5), cv::Vec2f upperRight = cv::Vec2f(2.0, -0.5),
						 cv::Vec2f lowerRight = cv::Vec2f(1.0, -0.5), cv::Vec2f lowerLeft = cv::Vec2f(1.0, 0.5));

	/* Commnuication */
	const bool connectTo(const int& port, const std::string& serverAddress);
	const bool sendMessage();

	/* Visualization Set */
	const bool drawRegionInColorImage(cv::Scalar regionColor = cv::Scalar(255, 255, 255));
	const bool drawBodiesInRegion(cv::Scalar bodyColor = cv::Scalar(255, 255, 255));
	const bool drawUserPosition();
	const bool drawUserPositionInRegion(cv::Scalar pointColor = cv::Scalar(255, 255, 255));

protected:
	/* Constructors for singleton pattern using */
		// Constructor main body
	BodyDetection();
	BodyDetection(BodyDetection const& that);	// Not implement
	void operator=(BodyDetection const& that);	// Not implement

	/* Region Description for Rectangle Shape */
		// Index 0 is the Z direction in Kinect Coordinate, and Index 1 is the X direction
	cv::Vec2f upperRightCorner;
	cv::Vec2f upperLeftCorner;
	cv::Vec2f lowerRightCorner;
	cv::Vec2f lowerLeftCorner;
		// Used to draw the rough detection region on the color image
	cv::Vec2i ulColorPoint, ulColorPointCeil;
	cv::Vec2i urColorPoint, urColorPointCeil;
	cv::Vec2i llColorPoint, llColorPointCeil;
	cv::Vec2i lrColorPoint, lrColorPointCeil;

	/* Variables for communication */
	TCPConnector					connector;
	vector< TCPStream >				streams;
	vector< bodyDetectedMessage >	messagesToSend;

	/* Variables for maintaining the user' states */
	std::vector< cv::Vec3f >			userPosition;
	std::vector< unsigned long long>	userID;
	std::vector< bool >					userInRegion;
	//std::vector< bool >				userWavingHand;

	/* Auxiliary function set */
	void prepareDrawRectangularRegion(const float yAxisUpperOffset = -0.5f, const float yAxisLowerOffset = -0.4f);

	void prepareMessageToSend();

	const bool checkBodyInRectangularRegion(const int bodyIndex, const cv::Vec3f& position);

	const cv::Vec3f extractUserPositionWithUpperBodyAverage(const int bodyIndex);
	const cv::Vec3f getUserSkeletonPosition(const int bodyIndex, const JointType jointName);

	const bool drawUserPoint(const int bodyIndex, cv::Scalar pointColor);
};

#endif //BODY_DETECTINO_RANGE_H