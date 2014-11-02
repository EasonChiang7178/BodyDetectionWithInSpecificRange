#ifndef KINECT2_EXCEPTIONS_H
#define KINECT2_EXCEPTIONS_H

/* Standrad Library Template */
#include <sstream>
#include <string>
#include <exception>

#include <Kinect.h>

namespace Kinect2 {
	/* Sensor open failed */
	class ExecuteSensorOpenFailed : public std::exception {
	public:
		virtual const char* what() const;
	};

	/* Sensor close failed*/
	class ExecuteSensorCloseFailed : public std::exception {
	public:
		ExecuteSensorCloseFailed(HRESULT hr);
		virtual const char* what() const;

	private:
		HRESULT hResult;
	};

	/* About the fail in Kinect stream manipulation */
	class ExecuteStreamManipulationFailed : public std::exception {
	public:
		ExecuteStreamManipulationFailed(HRESULT hr, std::string functionInfo);
		virtual const char* what() const;

	private:
		HRESULT hResult;
		std::string info;
	};

	/* The frame data is pending in the stream? */
	class FrameDataPendingFailed : public std::exception{
	public:
		FrameDataPendingFailed(std::string functionInfo);
		virtual const char* what() const;

	private:
		std::string info;
	};

	/* Function Not Implement! */
	class NotImplement : public std::exception {
	public:
		NotImplement(std::string functionInfo);
		virtual const char* what() const;

	private:
		std::string info;
	};
}

#endif //KINECT2_EXCEPTION_H