#ifndef BODY_DETECTION_REGION_H
#define BODY_DETECTION_REGION_H

#define PI 3.14159265

#include <cmath>

#include "K4Wv2ToOpenCV/K4Wv2ToOpenCV.h"

using namespace Kinect2;

class BodyDetection : public K4Wv2ToOpenCV {
public:
		// Constructor with Singleton Pattern
	static BodyDetection& getDefaultKinectSensor();
	
	void checkAllBodiesInRegion();

	const std::vector< cv::Vec3f >&				getUsersPosition() const;
	const std::vector< unsigned long long >&	getUsersID() const;
	const std::vector< bool >&					isUsersInRegion() const;
	//const std::vector< bool >&				isUsersWavingHand() const;

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
	cv::Vec2i ulColorPoint, ulColorPointCell;
	cv::Vec2i urColorPoint, urColorPointCell;
	cv::Vec2i llColorPoint, llColorPointCell;
	cv::Vec2i lrColorPoint, lrColorPointCell;

	/* Variables for maintaining the user' states */
	std::vector< cv::Vec3f >			userPosition;
	std::vector< unsigned long long>	userID;
	std::vector< bool >					userInRegion;
	//std::vector< bool >				userWavingHand;

	/* Auxiliary function set */
	void prepareDrawRectangularRegion(const float yAxisUpperOffset = -0.5f, const float yAxisLowerOffset = -0.4f);

	const bool checkBodyInRectangularRegion(const int bodyIndex, const cv::Vec3f& position);

	const cv::Vec3f extractUserPositionWithUpperBodyAverage(const int bodyIndex);
	const cv::Vec3f getUserSkeletonPosition(const int bodyIndex, const JointType jointName);

	const bool drawUserPoint(const int bodyIndex, cv::Scalar pointColor);
};

#endif //BODY_DETECTINO_RANGE_H