#ifndef BODY_DETECTION_REGION_H
#define BODY_DETECTION_REGION_H

#include "K4Wv2ToOpenCV/K4Wv2ToOpenCV.h"

using namespace Kinect2;

class BodyDetection : public K4Wv2ToOpenCV {
public:
		// Constructor with Singleton Pattern
	static BodyDetection& getDefaultKinectSensor();
	
	const bool drawDetectionRegionInColorImage(cv::Scalar regionColor = cv::Scalar(255, 255, 255));

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
	cv::Vec2i ulColorPoint;
	cv::Vec2i urColorPoint;
	cv::Vec2i llColorPoint;
	cv::Vec2i lrColorPoint;

	/* Auxiliary function set */
	void preparingDrawRegion(const float yAxisUpperOffset = -0.3f, const float yAxisLowerOffset = -0.4f);
};

#endif //BODY_DETECTINO_RANGE_H