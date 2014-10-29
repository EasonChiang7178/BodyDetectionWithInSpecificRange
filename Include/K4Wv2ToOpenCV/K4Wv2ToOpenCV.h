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