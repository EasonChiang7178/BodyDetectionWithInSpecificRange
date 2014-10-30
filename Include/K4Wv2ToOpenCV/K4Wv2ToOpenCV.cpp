#include "K4Wv2ToOpenCV.h"

namespace Kinect2 {
	cv::Vec2f to_CV_Vec2f(const PointF& v) {
		return cv::Vec2f(v.X, v.Y);
	}

	cv::Vec2f to_CV_Vec2f(const ColorSpacePoint& v) {
		return cv::Vec2f(v.X, v.Y);
	}

	cv::Vec2f to_CV_Vec2f(const DepthSpacePoint& v) {
		return cv::Vec2f(v.X, v.Y);
	}

	cv::Vec3f to_CV_Vec3f(const CameraSpacePoint& v) {
		return cv::Vec3f(v.X, v.Y, v.Z);
	}

	cv::Vec4f to_CV_Vec4f(const Vector4& v) {
		return cv::Vec4f(v.w, v.x, v.y, v.z);
	}

	ColorSpacePoint	toColorSpacePoint(const cv::Vec2f& v) {
		ColorSpacePoint p;
		p.X = v[0];
		p.Y = v[1];
		return p;
	}

	DepthSpacePoint	toDepthSpacePoint(const cv::Vec2f& v) {
		DepthSpacePoint p;
		p.X = v[0];
		p.Y = v[1];
		return p;
	}

	CameraSpacePoint toCameraSpacePoint(const cv::Vec3f& v) {
		CameraSpacePoint p;
		p.X = v[0];
		p.Y = v[1];
		p.Z = v[2];
		return p;
	}

	Hand::Hand()
	: confidence(TrackingConfidence_Low), state(HandState_Unknown)
	{}
	
	TrackingConfidence Hand::getConfidence() const {
		return confidence;
	}
	
	HandState Hand::getHandState() const {
		return state;
	}
	
	
	
	Joint::Joint()
	: orientation( cv::Vec4f() ), parentJoint( JointType::JointType_Count ), position( cv::Vec3f() ), 
	  trackingState(TrackingState_NotTracked)
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
		return orientation;
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
			static std::map< JointType, float > weights;
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
			for (std::map<JointType, Joint>::const_iterator iter = jointMap.begin(); iter != jointMap.end(); ++iter ) {
				if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
					c += weights[ iter->first ];
				}
			}
		} else {
			for (std::map<JointType, Joint>::const_iterator iter = jointMap.begin(); iter != jointMap.end(); ++iter ) {
				if ( iter->second.getTrackingState() == TrackingState::TrackingState_Tracked ) {
					c += 1.0f;
				}
			}
			c /= (float)JointType::JointType_Count;
		}
		return c;
	}
	
	const std::map< Activity, DetectionResult >& Body::getActivities() const {
		return activities;
	}
	
	const std::map< Appearance, DetectionResult >& Body::getAppearances() const {
		return appearances;
	}
	
	const std::map< Expression, DetectionResult >& Body::getExpressions() const {
		return expressions;
	}
	
	const Hand& Body::getHandLeft() const {
		return hands[ 0 ];
	}
	
	const Hand& Body::getHandRight() const {
		return hands[ 1 ];
	}
	
	unsigned long long Body::getId() const { 
		return id; 
	}
	
	unsigned char Body::getIndex() const { 
		return index; 
	}
	
	const std::map< JointType, Joint >& Body::getJointMap() const {
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
	
	const long long Body::getTrackedTime() const {
		return (this->currentTimeStamp - this->startTimeStamp) / 1000000; // Get millisecond
	}
	
	Frame::Frame()
	: timeStamp(0L)
	{}
	
	long long Frame::getTimeStamp() const {
		return timeStamp;
	}
	
	void Frame::setTimeStamp(long long stamp) {
		this->timeStamp = stamp;
	}

	BodyFrame::BodyFrame()
	: Frame()
	{}
	
	const std::vector< Body >& BodyFrame::getBodies() const {
		return bodies;
	}

	void BodyFrame::resizeBodies(const int num) {
		this->bodies.resize(num);
	}

	//-------------------------------------------------//
	//---        Initialization Function Set        ---//
	//-------------------------------------------------//
					/** Constructors **/
	K4Wv2ToOpenCV& K4Wv2ToOpenCV::getDefaultKinectSensor() {
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


		skeletonDrawOrder[0] = std::make_pair<JointType, JointType>(JointType_Head, JointType_Neck);
		skeletonDrawOrder[1] = std::make_pair<JointType, JointType>(JointType_Neck, JointType_SpineShoulder);
		skeletonDrawOrder[2] = std::make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_SpineMid);
		skeletonDrawOrder[3] = std::make_pair<JointType, JointType>(JointType_SpineMid, JointType_SpineBase);
		skeletonDrawOrder[4] = std::make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_ShoulderRight);
		skeletonDrawOrder[5] = std::make_pair<JointType, JointType>(JointType_SpineShoulder, JointType_ShoulderLeft);
		skeletonDrawOrder[6] = std::make_pair<JointType, JointType>(JointType_SpineBase, JointType_HipRight);
		skeletonDrawOrder[7] = std::make_pair<JointType, JointType>(JointType_SpineBase, JointType_HipLeft);
			// Right Arm    
		skeletonDrawOrder[8] = std::make_pair<JointType, JointType>(JointType_ShoulderRight, JointType_ElbowRight);
		skeletonDrawOrder[9] = std::make_pair<JointType, JointType>(JointType_ElbowRight, JointType_WristRight);
		skeletonDrawOrder[10] = std::make_pair<JointType, JointType>(JointType_WristRight, JointType_HandRight);
		skeletonDrawOrder[11] = std::make_pair<JointType, JointType>(JointType_HandRight, JointType_HandTipRight);
		skeletonDrawOrder[12] = std::make_pair<JointType, JointType>(JointType_WristRight, JointType_ThumbRight);
			// Left Arm
		skeletonDrawOrder[13] = std::make_pair<JointType, JointType>(JointType_ShoulderLeft, JointType_ElbowLeft);
		skeletonDrawOrder[14] = std::make_pair<JointType, JointType>(JointType_ElbowLeft, JointType_WristLeft);
		skeletonDrawOrder[15] = std::make_pair<JointType, JointType>(JointType_WristLeft, JointType_HandLeft);
		skeletonDrawOrder[16] = std::make_pair<JointType, JointType>(JointType_HandLeft, JointType_HandTipLeft);
		skeletonDrawOrder[17] = std::make_pair<JointType, JointType>(JointType_WristLeft, JointType_ThumbLeft);
			// Right Leg
		skeletonDrawOrder[18] = std::make_pair<JointType, JointType>(JointType_HipRight, JointType_KneeRight);
		skeletonDrawOrder[19] = std::make_pair<JointType, JointType>(JointType_KneeRight, JointType_AnkleRight);
		skeletonDrawOrder[20] = std::make_pair<JointType, JointType>(JointType_AnkleRight, JointType_FootRight);
			// Left Leg
		skeletonDrawOrder[21] = std::make_pair<JointType, JointType>(JointType_HipLeft, JointType_KneeLeft);
		skeletonDrawOrder[22] = std::make_pair<JointType, JointType>(JointType_KneeLeft, JointType_AnkleLeft);
		skeletonDrawOrder[23] = std::make_pair<JointType, JointType>(JointType_AnkleLeft, JointType_FootLeft);
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

	// Body
	const bool K4Wv2ToOpenCV::initializeBodyStream(bool enableJointTracking, bool enableHandTracking) {
		this->enabledJointTracking = enableJointTracking;
		this->enabledHandTracking = enableHandTracking;
		
		if (this->enabledBodyStream == true) {
			std::cout << "> [INFO] BodyStream is already opened" << std::endl;
			return false;
		}

		//kcbBodyFrame = new KCBBodyFrame();
		//kcbBodyFrame->Bodies = new IBody*[BODY_COUNT];
		//for (int i = 0; i < BODY_COUNT; i++)
		//	kcbBodyFrame->Bodies[i] = nullptr;

		cvBodyFrame.resizeBodies(BODY_COUNT);

		this->enabledBodyStream = true;
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
			//for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
			//	if (KCBIsFrameReady(KCBKinectHandle, FrameSourceTypes_Color) == true)
			//		break;
			//	Sleep(this->tryToReadDataInterval);
			//}
			//hr = KCBGetColorFrame(KCBKinectHandle, kcbColorFrame);

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
			/* Get raw data from KCBv2, try with specific times and interval */
			BodyFrame frame;
			long long currentTimeStamp = 0L;
			IBody* kinectBodies[BODY_COUNT] = { 0 };

			for (int tryCounter = this->tryToReadDataTimes; tryCounter > 0; tryCounter--) {
				hr = KCBGetBodyData(KCBKinectHandle, BODY_COUNT, kinectBodies, &currentTimeStamp);

				if (hr != E_PENDING)
					break;
				Sleep(this->tryToReadDataInterval);
			}

			if (hr == E_PENDING)
				throw FrameDataPendingFailed("updata body data");
			else if (FAILED(hr))
				throw ExecuteStreamManipulationFailed(hr, "updataStreamData-BodyStream");

			/* Convert the KCBBodyFrame information to BodyFrame object */
			for (unsigned char i = 0; i < BODY_COUNT; ++i) {
				IBody* kinectBody = kinectBodies[i];
				if (kinectBody != nullptr) {
					Body body;
					body.index = i;
					boolean isTracked = false;
					hr = kinectBody->get_IsTracked(&isTracked);
					if (SUCCEEDED(hr) && isTracked) {
						body.tracked = true;

						/* Time stamp for tracked body */
						if (cvBodyFrame.bodies[i].startTimeStamp == 0L)
							body.startTimeStamp = currentTimeStamp;
						else
							body.startTimeStamp = cvBodyFrame.bodies[i].startTimeStamp;
						body.currentTimeStamp = currentTimeStamp;

						if (enabledJointTracking == true) {
							::Joint joints[JointType_Count];
							kinectBody->GetJoints(JointType_Count, joints);

							JointOrientation jointOrientations[JointType_Count];
							kinectBody->GetJointOrientations(JointType_Count, jointOrientations);

							for (int j = 0; j < JointType_Count; ++j) {
								JointType parentJoint = (JointType)j;
								switch ((JointType)j) {
									case JointType::JointType_AnkleLeft:
										parentJoint = JointType_KneeLeft;
										break;
									case JointType::JointType_AnkleRight:
										parentJoint = JointType_KneeRight;
										break;
									case JointType::JointType_ElbowLeft:
										parentJoint = JointType_ShoulderLeft;
										break;
									case JointType::JointType_ElbowRight:
										parentJoint = JointType_ShoulderRight;
										break;
									case JointType::JointType_FootLeft:
										parentJoint = JointType_AnkleLeft;
										break;
									case JointType::JointType_FootRight:
										parentJoint = JointType_AnkleRight;
										break;
									case JointType::JointType_HandLeft:
										parentJoint = JointType_WristLeft;
										break;
									case JointType::JointType_HandRight:
										parentJoint = JointType_WristRight;
										break;
									case JointType::JointType_HandTipLeft:
										parentJoint = JointType_HandLeft;
										break;
									case JointType::JointType_HandTipRight:
										parentJoint = JointType_HandRight;
										break;
									case JointType::JointType_Head:
										parentJoint = JointType_Neck;
										break;
									case JointType::JointType_HipLeft:
										parentJoint = JointType_SpineBase;
										break;
									case JointType::JointType_HipRight:
										parentJoint = JointType_SpineBase;
										break;
									case JointType::JointType_KneeLeft:
										parentJoint = JointType_HipLeft;
										break;
									case JointType::JointType_KneeRight:
										parentJoint = JointType_HipRight;
										break;
									case JointType::JointType_Neck:
										parentJoint = JointType_SpineShoulder;
										break;
									case JointType::JointType_ShoulderLeft:
										parentJoint = JointType_SpineShoulder;
										break;
									case JointType::JointType_ShoulderRight:
										parentJoint = JointType_SpineShoulder;
										break;
									case JointType::JointType_SpineBase:
										parentJoint = JointType_SpineBase;
										break;
									case JointType::JointType_SpineMid:
										parentJoint = JointType_SpineBase;
										break;
									case JointType::JointType_SpineShoulder:
										parentJoint = JointType_SpineMid;
										break;
									case JointType::JointType_ThumbLeft:
										parentJoint = JointType_HandLeft;
										break;
									case JointType::JointType_ThumbRight:
										parentJoint = JointType_HandRight;
										break;
									case JointType::JointType_WristLeft:
										parentJoint = JointType_ElbowLeft;
										break;
									case JointType::JointType_WristRight:
										parentJoint = JointType_ElbowRight;
										break;
								}

								Joint joint(
									to_CV_Vec3f(joints[j].Position),
									to_CV_Vec4f(jointOrientations[j].Orientation),
									joints[j].TrackingState,
									parentJoint
									);

								body.jointMap.insert(std::pair<JointType, Joint>(static_cast<JointType>(j), joint));
							}
						}

						PointF lean;
						kinectBody->get_Engaged(&body.engaged);
						kinectBody->get_Lean(&lean);
						kinectBody->get_LeanTrackingState(&body.leanTrackingState);
						kinectBody->get_TrackingId(&body.id);

						body.lean = to_CV_Vec2f(lean);

						DetectionResult activities[Activity_Count];
						kinectBody->GetActivityDetectionResults((UINT)Activity_Count, activities);
						for (size_t j = 0; j < (size_t)Activity_Count; ++j) {
							body.activities[(Activity)j] = activities[j];
						}

						DetectionResult appearances[Appearance_Count];
						kinectBody->GetAppearanceDetectionResults((UINT)Appearance_Count, appearances);
						for (size_t j = 0; j < (size_t)Appearance_Count; ++j) {
							body.appearances[(Appearance)i] = appearances[j];
						}

						DetectionResult expressions[Expression_Count];
						kinectBody->GetExpressionDetectionResults((UINT)Expression_Count, expressions);
						for (size_t j = 0; j < (size_t)Expression_Count; ++j) {
							body.expressions[(Expression)j] = expressions[j];
						}

						if (enabledHandTracking = true) {
							kinectBody->get_HandLeftConfidence(&body.hands[0].confidence);
							kinectBody->get_HandLeftState(&body.hands[0].state);
							kinectBody->get_HandRightConfidence(&body.hands[1].confidence);
							kinectBody->get_HandRightState(&body.hands[1].state);
						}
					} else { // if isTracked equal to false
						body.currentTimeStamp = 0L;
						body.startTimeStamp = 0L;
						body.tracked = false;
					}

					kinectBody->Release();
					kinectBody = nullptr;

					frame.bodies.push_back(body);
				}
			}
			frame.timeStamp = static_cast<long long>(currentTimeStamp);
			if (frame.getTimeStamp() > cvBodyFrame.getTimeStamp())
				cvBodyFrame = frame;
		}

			// Audio
		//TODO
			// MultiStream
		//TODO

		return true;
	}

	void K4Wv2ToOpenCV::drawBodySkeletonInColorImage() {
		for (int detectedBody = 0; detectedBody < BODY_COUNT; detectedBody++) {
			if (cvBodyFrame.getBodies()[detectedBody].isTracked() == false)
				continue;

			std::map< JointType, Joint >& cameraJointMap = cvBodyFrame.bodies[detectedBody].jointMap;
			std::map< JointType, cv::Vec2i > colorJointMap;

			for (auto cameraJointMapIter = cameraJointMap.begin(); cameraJointMapIter != cameraJointMap.end(); cameraJointMapIter++)
				colorJointMap.insert(std::pair< JointType, cv::Vec2i >(cameraJointMapIter->first, mapCameraToColor(cameraJointMapIter->second.position)));

			/* Draw Lines */
			for (int i = 0; i < JointType_Count; i++) {
				cv::Point p1(colorJointMap[skeletonDrawOrder[i].first]);
				cv::Point p2(colorJointMap[skeletonDrawOrder[i].second]);

				if (p1.x > 0 && p1.y > 0 && p2.x > 0 && p2.y > 0) {
					TrackingState p1State = cameraJointMap[skeletonDrawOrder[i].first].trackingState;
					TrackingState p2State = cameraJointMap[skeletonDrawOrder[i].second].trackingState;

					if (p1State == TrackingState_Inferred || p2State == TrackingState_Inferred)
						cv::line(cvColorMat, cv::Point(colorJointMap[skeletonDrawOrder[i].first]), cv::Point(colorJointMap[skeletonDrawOrder[i].second]), static_cast<cv::Scalar>(this->colorizeBody(detectedBody)), 1, 8);
					else
						cv::line(cvColorMat, cv::Point(colorJointMap[skeletonDrawOrder[i].first]), cv::Point(colorJointMap[skeletonDrawOrder[i].second]), static_cast<cv::Scalar>(this->colorizeBody(detectedBody)), 7, CV_AA);
				}
			}

			/* Draw Joints */
			for (auto colorJointMapIter = colorJointMap.begin(); colorJointMapIter != colorJointMap.end(); colorJointMapIter++)
				if ((colorJointMapIter->second[0] > 0) && (colorJointMapIter->second[0] < cvColorMat.cols) && (colorJointMapIter->second[1] > 0) && (colorJointMapIter->second[1] < cvColorMat.rows)) {
					TrackingState p = cameraJointMap[colorJointMapIter->first].trackingState;

					if (p == TrackingState_Inferred)
						cv::circle(cvColorMat, cv::Point(colorJointMapIter->second), 3, cv::Scalar(255, 102, 187), CV_FILLED, CV_AA);
					else if (p == TrackingState_Tracked)
						cv::circle(cvColorMat, cv::Point(colorJointMapIter->second), 8, cv::Scalar(255, 0, 127), CV_FILLED, CV_AA);
				}
		}
	}

	void K4Wv2ToOpenCV::drawBodySkeletonInDepthImage() {
		for (int detectedBody = 0; detectedBody < BODY_COUNT; detectedBody++) {
			if (cvBodyFrame.getBodies()[detectedBody].isTracked() == false)
				continue;

			std::map< JointType, Joint >& cameraJointMap = cvBodyFrame.bodies[detectedBody].jointMap;
			std::map< JointType, cv::Vec2i > depthJointMap;

			for (auto cameraJointMapIter = cameraJointMap.begin(); cameraJointMapIter != cameraJointMap.end(); cameraJointMapIter++)
				depthJointMap.insert(std::pair< JointType, cv::Vec2i >(cameraJointMapIter->first, mapCameraToDepth(cameraJointMapIter->second.position)));

			/* Draw Lines */
			for (int i = 0; i < JointType_Count; i++) {
				cv::Point p1(depthJointMap[skeletonDrawOrder[i].first]);
				cv::Point p2(depthJointMap[skeletonDrawOrder[i].second]);

				if (p1.x > 0 && p1.y > 0 && p2.x > 0 && p2.y > 0) {
					TrackingState p1State = cameraJointMap[skeletonDrawOrder[i].first].trackingState;
					TrackingState p2State = cameraJointMap[skeletonDrawOrder[i].second].trackingState;

					if (p1State == TrackingState_Inferred || p2State == TrackingState_Inferred)
						cv::line(cvDepthMat, cv::Point(depthJointMap[skeletonDrawOrder[i].first]), cv::Point(depthJointMap[skeletonDrawOrder[i].second]), static_cast<cv::Scalar>(this->colorizeBody(detectedBody)), 1, 8);
					else
						cv::line(cvDepthMat, cv::Point(depthJointMap[skeletonDrawOrder[i].first]), cv::Point(depthJointMap[skeletonDrawOrder[i].second]), static_cast<cv::Scalar>(this->colorizeBody(detectedBody)), 7, CV_AA);
				}
			}

			/* Draw Joints */
			for (auto depthJointMapIter = depthJointMap.begin(); depthJointMapIter != depthJointMap.end(); depthJointMapIter++)
				if ((depthJointMapIter->second[0] > 0) && (depthJointMapIter->second[0] < cvDepthMat.cols) && (depthJointMapIter->second[1] > 0) && (depthJointMapIter->second[1] < cvDepthMat.rows)) {
				TrackingState p = cameraJointMap[depthJointMapIter->first].trackingState;

				if (p == TrackingState_Inferred)
					cv::circle(cvDepthMat, cv::Point(depthJointMapIter->second), 3, cv::Scalar(255, 102, 187), CV_FILLED, CV_AA);
				else if (p == TrackingState_Tracked)
					cv::circle(cvDepthMat, cv::Point(depthJointMapIter->second), 8, cv::Scalar(255, 0, 127), CV_FILLED, CV_AA);
				}
		}
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

		for (unsigned int index = 0; index < kcbInfraredFrame->Size; index++) {
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
		for (unsigned int index = 0; index < kcbBodyIndexFrame->Size; index++)
			*cvVisualizedBodyIndexPtr++ = colorizeBody(*cvBodyIndexMatPtr++);

		return this->cvVisualizedBodyIndex;
	}

	const BodyFrame& K4Wv2ToOpenCV::getBodyFrame() const {
		return cvBodyFrame;
	}

	//-------------------------------------------------//
	//---        Kinect Behavior Checking Set       ---//
	//-------------------------------------------------//
	const bool K4Wv2ToOpenCV::isKinectOpened() const {
		bool sensorIsOpen = false;
		if (KCBKinectHandle != KCB_INVALID_HANDLE)
			sensorIsOpen = true;

		return true;
	}

	const bool K4Wv2ToOpenCV::isColorStreamEnabled() const {
		return enabledColorStream;
	}
	const bool K4Wv2ToOpenCV::isInfraredStreamEnabled() const {
		return enabledInfraredStream;
	}
	const bool K4Wv2ToOpenCV::isDepthStreamEnabled() const {
		return enabledDepthStream;
	}
	const bool K4Wv2ToOpenCV::isBodyIndexStreamEnabled() const {
		return enabledBodyIndexStream;
	}
	const bool K4Wv2ToOpenCV::isBodyStreamEnabled() const {
		return enabledBodyStream;
	}
	const bool K4Wv2ToOpenCV::isAudioStreamEnabled() const {
		return enabledAudioStream;
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
		//if (enabledBodyStream == true)		KCBReleaseBodyFrame(&kcbBodyFrame);
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

	cv::Vec2i K4Wv2ToOpenCV::mapCameraToColor(const cv::Vec3f& v) const {
		ColorSpacePoint p;
		KCBMapCameraPointToColorSpace(KCBKinectHandle, toCameraSpacePoint(v), &p);
		return cv::Vec2i(to_CV_Vec2f(p));
	}

	std::vector< cv::Vec2i > K4Wv2ToOpenCV::mapCameraToColor(const std::vector< cv::Vec3f >& v) const {
		std::vector< cv::Vec2i > p;
		std::vector< CameraSpacePoint > camera;
		std::vector< ColorSpacePoint > color(v.size());
		for_each(v.begin(), v.end(), [&camera](const cv::Vec3f& i) {
			camera.push_back(toCameraSpacePoint(i));
		});

		KCBMapCameraPointsToColorSpace(KCBKinectHandle, camera.size(), &camera[0], color.size(), &color[0]);
		for_each(color.begin(), color.end(), [&p](const ColorSpacePoint& i) {
			p.push_back(cv::Vec2i(to_CV_Vec2f(i)));
		});
		return p;
	}

	cv::Vec2i K4Wv2ToOpenCV::mapCameraToDepth(const cv::Vec3f& v) const {
		DepthSpacePoint p;
		KCBMapCameraPointToDepthSpace(KCBKinectHandle, toCameraSpacePoint(v), &p);
		return cv::Vec2i(to_CV_Vec2f(p));
	}

	std::vector< cv::Vec2i > K4Wv2ToOpenCV::mapCameraToDepth(const std::vector< cv::Vec3f >& v) const {
		std::vector< cv::Vec2i > p;
		std::vector< CameraSpacePoint > camera;
		std::vector< DepthSpacePoint > depth(v.size());
		for_each(v.begin(), v.end(), [&camera](const cv::Vec3f& i) {
			camera.push_back(toCameraSpacePoint(i));
		});

		KCBMapCameraPointsToDepthSpace(KCBKinectHandle, camera.size(), &camera[0], depth.size(), &depth[0]);
		for_each(depth.begin(), depth.end(), [&p](const DepthSpacePoint& i) {
			p.push_back(cv::Vec2i(to_CV_Vec2f(i)));
		});
		return p;
	}

	//-------------------------------------------------//
	//---			 Auxiliary function set		    ---//
	//-------------------------------------------------//

	cv::Vec3b K4Wv2ToOpenCV::colorizeBody(const unsigned char& bodyIndex) {
		switch (bodyIndex) {
			case 255:
				return cv::Vec3b(0x00, 0x00, 0x00);
			case 0:
				return cv::Vec3b(0xFF, 0x00, 0x00);
			case 1:
				return cv::Vec3b(0x00, 0xFF, 0x00);
			case 2:
				return cv::Vec3b(0x00, 0x00, 0xFF);
			case 3:
				return cv::Vec3b(0xFF, 0xFF, 0x00);
			case 4:
				return cv::Vec3b(0x00, 0xFF, 0xFF);
			case 5:
				return cv::Vec3b(0xFF, 0xFF, 0x00);
			default:
				return cv::Vec3b(0xFF, 0xFF, 0xFF);
		}
	}
}