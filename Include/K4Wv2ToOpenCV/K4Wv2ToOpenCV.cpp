#include "K4Wv2ToOpenCV.h"

namespace Kinect2 {

	//-------------------------------------------------//
	//---        Initialization Function Set        ---//
	//-------------------------------------------------//
					/** Constructors **/
	K4Wv2ToOpenCV& K4Wv2ToOpenCV::GetDefaultKinectSensor() {
		static K4Wv2ToOpenCV instance;

		return instance;
	}

	K4Wv2ToOpenCV::K4Wv2ToOpenCV()
		: enabledColorStream(false), enabledInfraredStream(false), enabledDepthStream(false),
		  enabledBodyIndexStream(false), enabledBodyStream(false), enabledAudioStream(false),
		  KCBKinectHandle(KCB_INVALID_HANDLE)
	{
		/* Open the Kinect Sensor */
		KCBKinectHandle = KCBOpenDefaultSensor();
		if (KCBKinectHandle == KCB_INVALID_HANDLE) {
			throw ExecuteSensorOpenFailed();
		}

		/* Set up the configuration for this sensor */
		this->setTryToReadingDataInterval();
		this->setTryToReadingDataTimes();
	}

					/** Stream Initilizer **/
	const bool K4Wv2ToOpenCV::initializeColorStream(ColorImageFormat colorFormat) {
		if (enabledColorStream == true) {
			std::cout << "> [INFO] ColorStream is already opened" << std::endl;
			return false;
		}
		
		HRESULT hr = KCBGetColorFrameDescription(KCBKinectHandle, colorFormat, &kcbFrameDescription);
		if (SUCCEEDED(hr))
		{
			/* Set up the interface to KCBv2 */
			kcbColorFrame = new KCBColorFrame();
			kcbColorFrame->Size = kcbFrameDescription.bytesPerPixel * kcbFrameDescription.lengthInPixels;
			kcbColorFrame->Format = colorFormat;
			kcbColorFrame->Buffer = new BYTE[kcbColorFrame->Size];

			/* Set up the interface to OpenCV */
			cvColorMat = cv::Mat::zeros(cv::Size(kcbFrameDescription.width, kcbFrameDescription.height), CV_8UC4);
		}
		else {
			throw ExecuteStreamManipulationFailed(hr, "initializeColorStream()");
		}

		this->enabledColorStream = true;
		return true;
	}

	//-------------------------------------------------//
	//---        Configuration Function Set         ---//
	//-------------------------------------------------//

	const bool K4Wv2ToOpenCV::setTryToReadingDataInterval(const unsigned short interval) {
		this->tryToReadDataInterval = interval;
		return true;
	}

	const bool K4Wv2ToOpenCV::setTryToReadingDataTimes(const unsigned short times) {
		this->tryToReadDataTimes = times;
		return true;
	}



	const bool K4Wv2ToOpenCV::updataStreamData() {
		if (KCBKinectHandle == KCB_INVALID_HANDLE) {
			throw ExecuteSensorOpenFailed();
			return false;
		}

		HRESULT hr;
			// Color
		if (enabledColorStream == true) {
			/* Get raw data from KCBv2, try with specific times and interval */
			int tryCounter = this->tryToReadDataTimes;
			do {
				Sleep(this->tryToReadDataInterval);
				hr = KCBGetColorFrame(KCBKinectHandle, kcbColorFrame);
			} while (tryCounter-- > 0 && hr == E_PENDING);

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata color data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-ColorStream");
			
			/* Convert the KCBv2 format to cv::Mat */
			unsigned char* cvColorMatPtr = cvColorMat.ptr< unsigned char >(0);
			for (int index = 0; index < kcbColorFrame->Size; index++) {
				cvColorMatPtr[index] = kcbColorFrame->Buffer[index];
			}
		}

			// Infrared
		if (enabledInfraredStream == true) {
			hr = KCBGetInfraredFrame(KCBKinectHandle, kcbInfraredFrame);
			if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-InfraredStream");
		}

			// Depth
		if (enabledDepthStream == true) {
			hr = KCBGetDepthFrame(KCBKinectHandle, kcbDepthFrame);
			if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-DepthStream");
		}

			// BodyIndex
		if (enabledBodyStream == true) {
			hr = KCBGetBodyIndexFrame(KCBKinectHandle, kcbBodyIndexFrame);
			if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-BodyIndexStream");
		}

			// Body
		if (enabledBodyStream == true) {
			hr = KCBGetBodyFrame(KCBKinectHandle, kcbBodyFrame);
			if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-BodyStream");
			//TODO
		}

			// Audio
		//TODO
			// MultiStream
		//TODO

		return true;
	}

	const cv::Mat& K4Wv2ToOpenCV::getColorImage() const {
		return cvColorMat;
	}

	void K4Wv2ToOpenCV::closeKinectSensor() {
		this->~K4Wv2ToOpenCV();
	}

	K4Wv2ToOpenCV::~K4Wv2ToOpenCV() {
		/* Closing the kinect2 sensor... */
		if (KCBKinectHandle != KCB_INVALID_HANDLE) {
			HRESULT hr = KCBCloseSensor(&KCBKinectHandle);
			if (FAILED(hr)) {
				throw ExecuteSensorCloseFailed(hr);
			}
			else {
				KCBKinectHandle = KCB_INVALID_HANDLE;
			}
		}

		/* Release the resource used in KCBv2*/
		if (enabledColorStream == true)		KCBReleaseColorFrame(&kcbColorFrame);
		if (enabledInfraredStream == true)	KCBReleaseInfraredFrame(&kcbInfraredFrame);
		if (enabledDepthStream == true)		KCBReleaseDepthFrame(&kcbDepthFrame);
		if (enabledBodyIndexStream == true) KCBReleaseBodyIndexFrame(&kcbBodyIndexFrame);
		if (enabledBodyStream == true)		KCBReleaseBodyFrame(&kcbBodyFrame);
		//if (enabledAudioStream == true) KCBReleaseAudioFrame(&kcbAudioFrame); // No KCBv2 API to release?
	}
}