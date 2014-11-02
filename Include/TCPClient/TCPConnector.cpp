#include "TCPConnector.h"

TCPConnector::TCPConnector()
	: ports(0), IPAddresses(0)
{}

TCPStream& TCPConnector::connect(const int port, const std::string server) {
	/* Initialize Winsock */
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);

	if (iResult != 0)
		throw ExecuteWinSocketFailed("WSAStartup failed!", iResult);

	/* Interpret DNS host name to IP address */
	SOCKADDR_IN serverAddress;
	memset(&serverAddress, 0, sizeof(serverAddress));
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(port);

	if (interpretHostName(server, &serverAddress.sin_addr) != 0)
		inet_pton(AF_INET, server.c_str(), &(serverAddress.sin_addr));

	SOCKET socketDescriptor = INVALID_SOCKET;
	socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

	if (socketDescriptor == INVALID_SOCKET)
		throw CreateSocketFailed();

	iResult = ::connect(socketDescriptor, (SOCKADDR*) &serverAddress, sizeof(serverAddress));
	if (iResult == SOCKET_ERROR)
		throw ExecuteWinSocketFailed("Connect to server failed!", iResult);

	TCPStream* tcpStream = new TCPStream(socketDescriptor, &serverAddress);
	return *tcpStream;
}

std::vector< TCPStream >& TCPConnector::connectAll() {
	std::vector< TCPStream >* connections = new std::vector< TCPStream >();
	for (size_t index = 0; index < IPAddresses.size(); index++)
		connections->push_back(connect(ports[index], IPAddresses[index]));

	return *connections;
}

void TCPConnector::addConnection(const int port, const std::string IPAddress) {
	ports.push_back(static_cast< unsigned int >(port));
	IPAddresses.push_back(IPAddress);
}

void TCPConnector::clearAddressesContainer() {
	ports.clear();
	ports.resize(0);

	IPAddresses.clear();
	IPAddresses.resize(0);
}

int TCPConnector::interpretHostName(const std::string hostName, IN_ADDR* address_info) {
	ADDRINFOA* res = NULL;
 
	int result = getaddrinfo(hostName.c_str(), NULL, NULL, &res);
	if (result == 0) {
		memcpy(address_info, &((SOCKADDR_IN*) res->ai_addr)->sin_addr, sizeof(struct in_addr));
		freeaddrinfo(res);
	}
	return result;
}