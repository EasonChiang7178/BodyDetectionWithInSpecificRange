#ifndef K4WV2TOOPENCV_H
#define K4WV2TOOPENCV_H

/* Standrad Library Template */
#include <iostream>
#include <vector>
#include <map>

/* 3rd Party Libraries */
	// Kinect Common Bridge v2 for Windows
#include <KCBv2lib.h>
	// OpenCV
#include <opencv2/opencv.hpp>

#include "Exceptions.h"

namespace Kinect2 {

		// Basic Type for all frame data in this class
	class Frame
	{
	public:
		Frame();
	
		long long 		getTimeStamp() const;

	protected:
		long long		timeStamp;
	};

	/* Body data description */
	class Hand {
	public:
		Hand();

		TrackingConfidence	getConfidence() const;
		HandState			getHandState() const;

	protected:
		TrackingConfidence	confidence;
		HandState			state;
	};

	class Joint {
	public:
		Joint();
		
		uint64_t			getId() const;
		const cv::Vec4f&	getOrientation() const;
		const cv::Vec3f&	getPosition() const;
		JointType			getParentJoint() const;
		TrackingState		getTrackingState() const;

	protected:
		Joint( const cv::Vec3f& position, const cv::Vec4f& orientation, 
			TrackingState trackingState, JointType parentJoint);
		
		cv::Vec4f		orientation;
		JointType		parentJoint;
		ci::Vec3f		position;
		TrackingState	trackingState;
	};

	class Body {
	public:
		Body();
	
		float											calcConfidence( bool weighted = false ) const;
	
		uint64_t										getId() const;
		uint8_t											getIndex() const;

		bool											isRestricted() const;
		bool											isTracked() const;

		const std::map<JointType, Body::Joint>&			getJointMap() const;
		const Hand&										getHandLeft() const;
		const Hand&										getHandRight() const;

		DetectionResult									isEngaged() const;
		TrackingState									getLeanTrackingState() const;

		const std::map<Activity, DetectionResult>&		getActivities() const;
		const std::map<Appearance, DetectionResult>&	getAppearances() const;
		const std::map<Expression, DetectionResult>&	getExpressions() const;
		const cv::Vec2f&								getLean() const;

		const long long 								getTrackedTime() const;
		
	protected:
		uint64_t										id;
		uint8_t											index;

		std::map<JointType, Body::Joint>				jointMap;
		Hand											hands[ 2 ];

		cv::Vec2f										lean;
		TrackingState									leanTrackingState;
		bool											restricted;
		bool											tracked;

		DetectionResult									engaged;
		std::map<Activity, DetectionResult>				activities;
		std::map<Appearance, DetectionResult>			appearances;
		std::map<Expression, DetectionResult>			expressions;

		long long 										currentTimeStamp;
		long long 										startTimeStamp;
	};

	class BodyFrame : public Frame {
	public:
		BodyFrame();
		const std::vector< Body >& 	getBodies() const;

	protected:
		std::vector< Body >			bodies;
	};

	class K4Wv2ToOpenCV {

	public:
		/* Initialization Function Set */
			// Constructor with Singleton Pattern
		static K4Wv2ToOpenCV& GetDefaultKinectSensor();
			//
		const bool initializeColorStream(ColorImageFormat colorFormat = ColorImageFormat_Bgra);
			//
		const bool initializeInfraredStream();
			//
		const bool initializeDepthStream();
			//
		const bool initializeBodyIndexStream();
			//
		const bool initializeBodyStream();
			//TODO
		const bool initializeAudioStream();
			//TODO
		const bool initialzeLongExposureStream();
			//TODO
		const bool initializeMultiStream();
			// Renamed deconstructor, which used to release all the resource and close the sensor
		void closeKinectSensor();

		/* Configuration function Set */
		const bool setTryToReadingDataTimes(const unsigned short times = 2000);
		const bool setTryToReadingDataInterval(const unsigned short interval = 1);

			// To retrieve the data from initialized streams
		const bool updataStreamData();

		/* Accessor Set */
			// Color
		const cv::Mat& getColorImage() const;
			// Infrared
		const cv::Mat& getInfraredRawImage() const;
		const cv::Mat& getInfraredVisulizedImage();
			// Depth
		const cv::Mat& getDepthRawImage() const;
		const cv::Mat& getDepthVisualizedImage();
			// Body Index
		const cv::Mat& getBodyIndexRawImage() const;
		const cv::Mat& getBodyIndexVisualizedImage();

		/* Kinect Behavior Checking Set */
		const bool isKinectOpened();
		const bool isColorStreamEnabled();
		const bool isInfraredStreamEnabled();
		const bool isDepthStreamEnabled();
		const bool isBodyIndexStreamEnabled();
		const bool isBodyStreamEnabled();
		const bool isAudioStreamEnabled();

	private:
		/* Constructors for singleton pattern using */
			// Constructor main body
		K4Wv2ToOpenCV();
		K4Wv2ToOpenCV(K4Wv2ToOpenCV const& that);	// Not implement
		void operator=(K4Wv2ToOpenCV const& that);	// Not implement

			// Deconstructor
		~K4Wv2ToOpenCV();

		/* Configuration for Kinect for Windows To OpenCV */
		unsigned short	tryToReadDataTimes;
		unsigned short	tryToReadDataInterval;

		/* Manipulation items in Kinect Common Bridge ver. II */
			// Sensor handle, for KCBv2 manipulation
		KCBHANDLE KCBKinectHandle;
			// FrameDescription, used to get all kinds of stream description
		KCBFrameDescription kcbFrameDescription;
		
		KCBColorFrame*		kcbColorFrame;
		KCBDepthFrame*		kcbDepthFrame;
		KCBInfraredFrame*	kcbInfraredFrame;
		KCBBodyIndexFrame*	kcbBodyIndexFrame;
		KCBBodyFrame*		kcbBodyFrame;
		KCBAudioFrame*		kcbAudioFrame; //TODO

		/* Interface to OpenCV */
		cv::Mat cvColorMat;
		cv::Mat cvInfraredMat;
		cv::Mat cvDepthMat;
		cv::Mat cvBodyIndexMat;

		cv::Mat cvVisualizedInfraredMat;
		cv::Mat cvVisualizedDepthMat;
		cv::Mat cvVisualizedBodyIndex;

		/* Kinect Behavior Description */
		bool enabledColorStream;
		bool enabledInfraredStream;
		bool enabledDepthStream;
		bool enabledBodyIndexStream;
		bool enabledBodyStream;
		bool enabledAudioStream;	//TODO
		bool enabledMultiStream;	//TODO

		/* Auxiliary function set */
		cv::Vec3b colorizeBody(const unsigned char& bodyIndex);
	};
}

#endif //K4WV2TOOPENCV_H