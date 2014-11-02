#include "Kinect2Exceptions.h"

namespace Kinect2 {

//-----			ExecuteSensorOpenFailed			-----//
const char* ExecuteSensorOpenFailed::what() const {
	return "Unable to open the kinect sensor";
}

ExecuteSensorCloseFailed::ExecuteSensorCloseFailed(HRESULT hr)
	: hResult(hr)
{}

//-----			ExecuteSensorCloseFailed		-----//
const char* ExecuteSensorCloseFailed::what() const {
	std::string error_code;
	std::stringstream ss;
	ss << hResult; ss >> error_code;

	return ("Unable to close the kinect sensor, (Error code:" + error_code + ")").c_str();
}

//-----			ExecuteStreamManipulatoinFailed		-----//
const char* ExecuteStreamManipulationFailed::what() const {
	std::string error_name("E_UNKNOWN");
	switch (hResult) {
		case E_UNEXPECTED:		error_name = "E_UNEXPECTED"; break;
		case E_NOTIMPL:			error_name = "E_NOTIMPL"; break;
		case E_OUTOFMEMORY:		error_name = "E_OUTOFMEMORY"; break;
		case E_INVALIDARG:		error_name = "E_INVALIDARG"; break;
		case E_NOINTERFACE:		error_name = "E_NOINTERFACE"; break;
		case E_POINTER:			error_name = "E_POINTER"; break;
		case E_HANDLE:			error_name = "E_HANDLE"; break;
		case E_ABORT:			error_name = "E_ABORT"; break;
		case E_FAIL:			error_name = "E_FAIL"; break;
		case E_ACCESSDENIED:	error_name = "E_ACCESSDENIED"; break;
	}

	return ("Failed in " + info," (" + error_name + ")").c_str();
}

ExecuteStreamManipulationFailed::ExecuteStreamManipulationFailed(HRESULT hr, std::string functionInfo) {
	hResult = hr;
	info = functionInfo;
}

//-----			FrameDataPendingFailed	-----//
const char* FrameDataPendingFailed::what() const {
	return ("Data Pending in " + info).c_str();
}

FrameDataPendingFailed::FrameDataPendingFailed(std::string functionInfo) {
	info = functionInfo;
}

//-----			NotImplement			-----//
const char* NotImplement::what() const {
	return ("This function is still not implemented! (" + info).c_str();
}

NotImplement::NotImplement(std::string functionInfo) {
	info = functionInfo;
}

}