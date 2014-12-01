#ifndef TCPCONNECTOR_H
#define TCPCONNECTOR_H

#include <winsock2.h>
#include <ws2tcpip.h>

#include <string>
#include <vector>

#include "TCPStream.h"
#include "TCPExceptions.h"

class TCPConnector {
public:
	TCPConnector();

	TCPStream*						connect(const int port = 1000, const std::string IP = "localhost");
	std::vector< TCPStream* >&		connectAll();

	void addConnection(const int port = 1000, const std::string IP = "localhost");
	void clearAddressesContainer();

protected:
	int interpretHostName(const std::string hostName, IN_ADDR* address_info);

	std::vector< unsigned short >	ports;
	std::vector< std::string >		IPAddresses;
};

#endif //TCPCONNECTOR_H