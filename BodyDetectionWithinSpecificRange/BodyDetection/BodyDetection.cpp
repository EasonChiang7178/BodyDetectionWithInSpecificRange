#include "BodyDetection.h"

/** Constructors **/
BodyDetection& BodyDetection::getDefaultKinectSensor() {
	static BodyDetection instance;

	return instance;
}

BodyDetection::BodyDetection()
	:K4Wv2ToOpenCV(),
	 lowerLeftCorner(1.0, 0.75), lowerRightCorner(1.0, -0.75),
	 upperLeftCorner(2.0, 0.75), upperRightCorner(2.0, -0.75)
{}

const bool BodyDetection::drawDetectionRegionInColorImage(cv::Scalar regionColor) {
	preparingDrawRegion();

	cv::circle(cvColorMat, cv::Point(ulColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(urColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(llColorPoint), 5, regionColor, CV_FILLED, CV_AA);
	cv::circle(cvColorMat, cv::Point(lrColorPoint), 5, regionColor, CV_FILLED, CV_AA);

	cv::line(cvColorMat, cv::Point(ulColorPoint), cv::Point(urColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(ulColorPoint), cv::Point(llColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(urColorPoint), cv::Point(lrColorPoint), regionColor, 3, CV_AA);
	cv::line(cvColorMat, cv::Point(llColorPoint), cv::Point(lrColorPoint), regionColor, 3, CV_AA);

	return true;
}

void BodyDetection::preparingDrawRegion(const float yAxisUpperOffset, const float yAxisLowerOffset) {
	cv::Vec3f upperLeftCorner3d(upperLeftCorner[1], yAxisUpperOffset, upperLeftCorner[0]),
		upperRightCorner3d(upperRightCorner[1], yAxisUpperOffset, upperRightCorner[0]),
		lowerLeftCorner3d(lowerLeftCorner[1], yAxisLowerOffset, lowerLeftCorner[0]),
		lowerRightCorner3d(lowerRightCorner[1], yAxisLowerOffset, lowerRightCorner[0]);

	ulColorPoint = this->mapCameraToColor(upperLeftCorner3d);
	urColorPoint = this->mapCameraToColor(upperRightCorner3d);
	llColorPoint = this->mapCameraToColor(lowerLeftCorner3d);
	lrColorPoint = this->mapCameraToColor(lowerRightCorner3d);
}