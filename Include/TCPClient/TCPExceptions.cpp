#include "TCPExceptions.h"

ExecuteWinSocketFailed::ExecuteWinSocketFailed(const std::string& info, const int& code) {
	std::string error_function_info = info;
	
	int error_code = code;
	std::string error_code_str;

	std::stringstream ss;
	ss << error_code; ss >> error_code_str;

	error_info = error_function_info + " (Error code:" + error_code_str + ")";
}

const char* ExecuteWinSocketFailed::what() const {
	return error_info.c_str();
}

const char* CreateSocketFailed::what() const {
	return "Create Socket Failed";
}