#include "K4Wv2ToOpenCV.h"

namespace Kinect2 {

Hand::Hand()
: confidence(TrackingConfidence_Low), state(HandState_Unknown)
{}

TrackingConfidence Hand::getConfidence() const {
	return confidence;
}

HandState Hand::getState() const {
	return state;
}

Joint::Joint()
: orientation( cv::Vec4f() ), parentJoint( JointType::JointType_Count ), position( cv::Vec3f() ), 
  mTrackingState(TrackingState_NotTracked)
{}

Joint::Joint(const cv::Vec3f& position, const cv::Vec4f& orientation, TrackingState trackingState, JointType parentJoint )
: orientation( orientation ), position( position ), parentJoint( parentJoint ), 
trackingState( trackingState )
{}

JointType Joint::getParentJoint() const {
	return parentJoint;
}

const cv::Vec3f& Joint::getPosition() const {
	return position;
}

const cv::Vec4f& Joint::getOrientation() const {
	return mOrientation;
}

TrackingState Joint::getTrackingState() const {
	return trackingState;
}

Body::Body()
: engaged( DetectionResult_Unknown ), id( 0 ), index( 0 ), 
lean( cv::Vec2f() ), leanTrackingState( TrackingState_NotTracked ), 
restricted( false ), tracked( false ),
startTimeStamp(0L), currentTimeStamp(0L)
{
	for ( size_t i = 0; i < (size_t) Activity_Count; ++i )
		activities[ (Activity) i ] = DetectionResult_Unknown;
	for ( size_t i = 0; i < (size_t) Appearance_Count; ++i )
		appearances[ (Appearance) i ] = DetectionResult_Unknown;
	for ( size_t i = 0; i < (size_t) Expression_Count; ++i )
		expressions[ (Expression) i ] = DetectionResult_Unknown;
}

float Body::calcConfidence(bool weighted) const {
	float c = 0.0f;
	if (weighted) {
		static map< JointType, float > weights;
		if (weights.empty()) {
			weights[ JointType::JointType_SpineBase ]		= 0.042553191f;
			weights[ JointType::JointType_SpineMid ]		= 0.042553191f;
			weights[ JointType::JointType_Neck ]			= 0.021276596f;
			weights[ JointType::JointType_Head ]			= 0.042553191f;
			weights[ JointType::JointType_ShoulderLeft ]	= 0.021276596f;
			weights[ JointType::JointType_ElbowLeft ]		= 0.010638298f;
			weights[ JointType::JointType_WristLeft ]		= 0.005319149f;
			weights[ JointType::JointType_HandLeft ]		= 0.042553191f;
			weights[ JointType::JointType_ShoulderRight ]	= 0.021276596f;
			weights[ JointType::JointType_ElbowRight ]		= 0.010638298f;
			weights[ JointType::JointType_WristRight ]		= 0.005319149f;
			weights[ JointType::JointType_HandRight ]		= 0.042553191f;
			weights[ JointType::JointType_HipLeft ]			= 0.021276596f;
			weights[ JointType::JointType_KneeLeft ]		= 0.010638298f;
			weights[ JointType::JointType_AnkleLeft ]		= 0.005319149f;
			weights[ JointType::JointType_FootLeft ]		= 0.042553191f;
			weights[ JointType::JointType_HipRight ]		= 0.021276596f;
			weights[ JointType::JointType_KneeRight ]		= 0.010638298f;
			weights[ JointType::JointType_AnkleRight ]		= 0.005319149f;
			weights[ JointType::JointType_FootRight ]		= 0.042553191f;
			weights[ JointType::JointType_SpineShoulder ]	= 0.002659574f;
			weights[ JointType::JointType_HandTipLeft ]		= 0.002659574f;
			weights[ JointType::JointType_ThumbLeft ]		= 0.002659574f;
			weights[ JointType::JointType_HandTipRight ]	= 0.002659574f;
			weights[ JointType::JointType_ThumbRight ]		= 0.521276596f;
		}
		for ( map<JointType, Joint>::const_iterator iter = jointMap.begin(); iter != jointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += weights[ iter->first ];
			}
		}
	} else {
		for ( map<JointType, Joint>::const_iterator iter = jointMap.begin(); iter != jointMap.end(); ++iter ) {
			if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
				c += 1.0f;
			}
		}
		c /= (float)JointType::JointType_Count;
	}
	return c;
}

const map< Activity, DetectionResult >& Body::getActivities() const {
	return activities;
}

const map< Appearance, DetectionResult >& Body::getAppearances() const {
	return appearances;
}

const map< Expression, DetectionResult >& Body::getExpressions() const {
	return mExpressions;
}

const Body::Hand& Body::getHandLeft() const {
	return hands[ 0 ];
}

const Body::Hand& Body::getHandRight() const {
	return hands[ 1 ];
}

uint64_t Body::getId() const { 
	return id; 
}

uint8_t Body::getIndex() const { 
	return index; 
}

const map< JointType, Body::Joint >& Body::getJointMap() const { 
	return jointMap; 
}

const cv::Vec2f& Body::getLean() const {
	return lean;
}

TrackingState Body::getLeanTrackingState() const {
	return leanTrackingState;
}

DetectionResult Body::isEngaged() const {
	return engaged;
}

bool Body::isTracked() const { 
	return tracked; 
}

Frame::Frame()
: timeStamp(0L)
{}

long long Frame::getTimeStamp() const {
	return timeStamp;
}

BodyFrame::BodyFrame()
: Frame()
{}

const vector< Body >& BodyFrame::getBodies() const {
	return bodies;
}

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
	//Color
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

	//Infrared
	const bool K4Wv2ToOpenCV::initializeInfraredStream() {
		if (enabledInfraredStream == true) {
			std::cout << "> [INFO] InfraredStream is already opened" << std::endl;
			return false;
		}

		HRESULT hr = KCBGetInfraredFrameDescription(KCBKinectHandle, &kcbFrameDescription);
		if (SUCCEEDED(hr))
		{
			/* Set up the interface to KCBv2 */
			kcbInfraredFrame = new KCBInfraredFrame();
			kcbInfraredFrame->Size = kcbFrameDescription.lengthInPixels;
			kcbInfraredFrame->Buffer = new UINT16[kcbInfraredFrame->Size];

			/* Set up the interface to OpenCV */
			cvInfraredMat = cv::Mat::zeros(cv::Size(kcbFrameDescription.width, kcbFrameDescription.height), CV_16UC1);
		}
		else {
			throw ExecuteStreamManipulationFailed(hr, "initializeInfraredStream()");
		}

		this->enabledInfraredStream = true;
		return true;
	}

	//Depth
	const bool K4Wv2ToOpenCV::initializeDepthStream() {
		if (enabledDepthStream == true) {
			std::cout << "> [INFO] DepthStream is already opened" << std::endl;
			return false;
		}

		HRESULT hr = KCBGetDepthFrameDescription(KCBKinectHandle, &kcbFrameDescription);
		if (SUCCEEDED(hr))
		{
			/* Set up the interface to KCBv2 */
			kcbDepthFrame = new KCBDepthFrame();
			kcbDepthFrame->Size = kcbFrameDescription.lengthInPixels;
			kcbDepthFrame->Buffer = new UINT16[kcbDepthFrame->Size];

			/* Set up the interface to OpenCV */
			cvDepthMat = cv::Mat::zeros(cv::Size(kcbFrameDescription.width, kcbFrameDescription.height), CV_16UC1);
		}
		else {
			throw ExecuteStreamManipulationFailed(hr, "initializeDepthStream()");
		}

		this->enabledDepthStream = true;
		return true;
	}

	//BodyIndex
	const bool K4Wv2ToOpenCV::initializeBodyIndexStream() {
		if (enabledBodyIndexStream == true) {
			std::cout << "> [INFO] BodyIndexStream is already opened" << std::endl;
			return false;
		}

		HRESULT hr = KCBGetBodyIndexFrameDescription(KCBKinectHandle, &kcbFrameDescription);
		if (SUCCEEDED(hr))
		{
			/* Set up the interface to KCBv2 */
			kcbBodyIndexFrame = new KCBBodyIndexFrame();
			kcbBodyIndexFrame->Size = kcbFrameDescription.bytesPerPixel * kcbFrameDescription.lengthInPixels;
			kcbBodyIndexFrame->Buffer = new BYTE[kcbBodyIndexFrame->Size];

			/* Set up the interface to OpenCV */
			cvBodyIndexMat = cv::Mat::zeros(cv::Size(kcbFrameDescription.width, kcbFrameDescription.height), CV_8UC1);
		}
		else {
			throw ExecuteStreamManipulationFailed(hr, "initializeBodyIndexStream()");
		}

		this->enabledBodyIndexStream = true;
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

		HRESULT hr = S_OK;
			// Color
		if (enabledColorStream == true) {
			/* Get raw data from KCBv2, try with specific times and interval */
			for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
				hr = KCBGetColorFrame(KCBKinectHandle, kcbColorFrame);

				if (hr != E_PENDING)
					break;
				Sleep(this->tryToReadDataInterval);
			}

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata color data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-ColorStream");
			
			/* Convert the KCBv2 format to cv::Mat */
			unsigned char* cvColorMatPtr = cvColorMat.ptr< unsigned char >(0);
			for (unsigned int index = 0; index < kcbColorFrame->Size; index++) {
				cvColorMatPtr[index] = kcbColorFrame->Buffer[index];
			}
		}

			// Infrared
		if (enabledInfraredStream == true) {
			/* Get raw data from KCBv2, try with specific times and interval */
			for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
				hr = KCBGetInfraredFrame(KCBKinectHandle, kcbInfraredFrame);

				if (hr != E_PENDING)
					break;
				Sleep(this->tryToReadDataInterval);
			}

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata infrared data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-InfraredStream");

			/* Convert the KCBv2 format to cv::Mat */
			unsigned short* cvInfraredMatPtr = cvInfraredMat.ptr< unsigned short >(0);
			for (unsigned int index = 0; index < kcbInfraredFrame->Size; index++) {
				cvInfraredMatPtr[index] = kcbInfraredFrame->Buffer[index];
			}
		}

			// Depth
		if (enabledDepthStream == true) {
			/* Get raw data from KCBv2, try with specific times and interval */
			for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
				hr = KCBGetDepthFrame(KCBKinectHandle, kcbDepthFrame);

				if (hr != E_PENDING)
					break;
				Sleep(this->tryToReadDataInterval);
			}

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata depth data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-DepthStream");

			/* Convert the KCBv2 format to cv::Mat */
			unsigned short* cvDepthMatPtr = cvDepthMat.ptr< unsigned short >(0);
			for (unsigned int index = 0; index < kcbDepthFrame->Size; index++) {
				cvDepthMatPtr[index] = kcbDepthFrame->Buffer[index];
			}
		}

			// BodyIndex
		if (enabledBodyIndexStream == true) {
			/* Get raw data from KCBv2, try with specific times and interval */
			for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
				hr = KCBGetBodyIndexFrame(KCBKinectHandle, kcbBodyIndexFrame);

				if (hr != E_PENDING)
					break;
				Sleep(this->tryToReadDataInterval);
			}

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata body index data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-BodyIndexStream");

			/* Convert the KCBv2 format to cv::Mat */
			unsigned char* cvBodyIndexMatPtr = cvBodyIndexMat.ptr< unsigned char >(0);
			for (unsigned int index = 0; index < kcbBodyIndexFrame->Size; index++) {
				cvBodyIndexMatPtr[index] = kcbBodyIndexFrame->Buffer[index];
			}
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


	//-------------------------------------------------//
	//---		        Accessor Set		        ---//
	//-------------------------------------------------//

	const cv::Mat& K4Wv2ToOpenCV::getColorImage() const {
		if (enabledColorStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "color stream is not intilized");

		return this->cvColorMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getInfraredRawImage() const {
		if (enabledInfraredStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "infrared stream is not intilized");

		return this->cvInfraredMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getInfraredVisulizedImage() {
		if (enabledInfraredStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "infrared stream is not intilized");

		cvVisualizedInfraredMat = cv::Mat::zeros(cvInfraredMat.rows, cvInfraredMat.cols, CV_8UC1);

		/* Const information for processing infraredImage */
		const float InfraredSourceValueMaximum = static_cast<float>(USHRT_MAX);
		const float InfraredOutputValueMinimum = 0.01f;
		const float InfraredOutputValueMaximum = 1.0f;
		const float InfraredSceneValueAverage = 0.08f;
		const float InfraredSceneStandardDeviation = 3.0f;

		unsigned char* cvVisualizedInfraredMatPtr = cvVisualizedInfraredMat.ptr< unsigned char >(0);
		unsigned short* cvInfraredMatPtr = cvInfraredMat.ptr< unsigned short >(0);

		for (int index = 0; index < kcbInfraredFrame->Size; index++) {
			float intensityRatio = static_cast<float>(*cvInfraredMatPtr) / InfraredSourceValueMaximum;
			intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviation;
			intensityRatio = InfraredOutputValueMaximum > intensityRatio ? intensityRatio : InfraredOutputValueMaximum;
			intensityRatio = InfraredOutputValueMinimum < intensityRatio ? intensityRatio : InfraredOutputValueMinimum;

			*cvVisualizedInfraredMatPtr = static_cast<unsigned char>(intensityRatio * 255.0f);

			cvVisualizedInfraredMatPtr++; cvInfraredMatPtr++;
		}

		return this->cvVisualizedInfraredMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getDepthRawImage() const {
		if (enabledDepthStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "depth stream is not intilized");

		return this->cvDepthMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getDepthVisualizedImage() {
		if (enabledDepthStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "depth stream is not intilized");

		cvVisualizedDepthMat = cv::Mat::zeros(cvDepthMat.rows, cvDepthMat.cols, CV_8UC1);
		cvDepthMat.convertTo(cvVisualizedDepthMat, CV_8UC1, 255.0f / 4500.0f, 0.0f);

		return this->cvVisualizedDepthMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getBodyIndexRawImage() const {
		if (enabledBodyIndexStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "body index stream is not intilized");

		return this->cvBodyIndexMat;
	}

	const cv::Mat& K4Wv2ToOpenCV::getBodyIndexVisualizedImage() {
		if (enabledBodyIndexStream == false)
			throw ExecuteStreamManipulationFailed(E_FAIL, "body index stream is not intilized");

		cvVisualizedBodyIndex = cv::Mat::zeros(cvBodyIndexMat.rows, cvBodyIndexMat.cols, CV_8UC3);

		cv::Vec3b* cvVisualizedBodyIndexPtr = cvVisualizedBodyIndex.ptr< cv::Vec3b >(0);
		unsigned char* cvBodyIndexMatPtr = cvBodyIndexMat.ptr< unsigned char >(0);
		for (int index = 0; index < kcbBodyIndexFrame->Size; index++)
			*cvVisualizedBodyIndexPtr++ = colorizeBody(*cvBodyIndexMatPtr++);

		return this->cvVisualizedBodyIndex;
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

		/* Release the resource used in KCBv2 */
		if (enabledColorStream == true)		KCBReleaseColorFrame(&kcbColorFrame);
		if (enabledInfraredStream == true)	KCBReleaseInfraredFrame(&kcbInfraredFrame);
		if (enabledDepthStream == true)		KCBReleaseDepthFrame(&kcbDepthFrame);
		if (enabledBodyIndexStream == true) KCBReleaseBodyIndexFrame(&kcbBodyIndexFrame);
		if (enabledBodyStream == true)		KCBReleaseBodyFrame(&kcbBodyFrame);
		//if (enabledAudioStream == true) KCBReleaseAudioFrame(&kcbAudioFrame); // No KCBv2 API to release?

		/* Release the resource used in OpenCV */
		if (cvColorMat.empty() == false)				cvColorMat.release();
		if (cvInfraredMat.empty() == false)				cvInfraredMat.release();
		if (cvVisualizedInfraredMat.empty() == false) 	cvVisualizedInfraredMat.release();
		if (cvDepthMat.empty() == false)				cvDepthMat.release();
		if (cvVisualizedDepthMat.empty() == false)		cvVisualizedDepthMat.release();
		if (cvBodyIndexMat.empty() == false)			cvBodyIndexMat.release();
		if (cvVisualizedBodyIndex.empty() == false)		cvVisualizedBodyIndex.release();
	}


	//-------------------------------------------------//
	//---			 Auxiliary function set		    ---//
	//-------------------------------------------------//

	cv::Vec3b K4Wv2ToOpenCV::colorizeBody(const unsigned char& bodyIndex) {
		switch (bodyIndex) {
			case 255:
				return cv::Vec3b(0x00, 0x00, 0x00);
			case 1:
				return cv::Vec3b(0xFF, 0x00, 0x00);
			case 2:
				return cv::Vec3b(0x00, 0xFF, 0x00);
			case 3:
				return cv::Vec3b(0x00, 0x00, 0xFF);
			case 4:
				return cv::Vec3b(0xFF, 0xFF, 0x00);
			case 5:
				return cv::Vec3b(0x00, 0xFF, 0xFF);
			case 6:
				return cv::Vec3b(0xFF, 0xFF, 0x00);
			default:
				return cv::Vec3b(0xFF, 0xFF, 0xFF);
		}
	}
}