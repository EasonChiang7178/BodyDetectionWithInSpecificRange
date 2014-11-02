#ifndef TCP_EXCEPTIONS_H
#define TCP_EXCEPTIONS_H

#include <exception>
#include <sstream>
#include <string>

class ExecuteWinSocketFailed : public std::exception {
public:
	ExecuteWinSocketFailed(const std::string& info, const int& code);
	virtual const char* what() const;

private:
	std::string		error_info;
	int				error_code;
};

class CreateSocketFailed : public std::exception {
public:
	virtual const char* what() const;
};

#endif // TCP_EXCEPTIONS_H