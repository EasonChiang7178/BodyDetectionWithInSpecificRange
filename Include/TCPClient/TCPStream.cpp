#include "TCPStream.h"

std::string message::getMessage() const {
	return messageToSend;
}

TCPStream::TCPStream(int sd, struct sockaddr_in* address) : socketDescriptor(sd) {
	char ip[50];
	inet_ntop(PF_INET, (struct in_addr*)&(address->sin_addr.s_addr), ip, sizeof(ip)-1);
	peerIP = ip;
	peerPort = ntohs(address->sin_port);
}

TCPStream::~TCPStream() {
	closesocket(socketDescriptor);
	WSACleanup();
}

const bool TCPStream::send(message& datumToSend, const unsigned int delayTime) {
	datumToSend.makeMessage();

	int iResult = ::send(socketDescriptor, datumToSend.getMessage().c_str(), datumToSend.getMessage().size() + 1, 0);
	if (iResult == SOCKET_ERROR) {
		throw ExecuteWinSocketFailed("Send Failed!", iResult);
		return false;
	}

	Sleep(delayTime);
	return true;
}

const bool TCPStream::send(const unsigned int delayTime) {
	if (data.size() == 0)
		return false;

	for (auto datumToSend = data.begin(); datumToSend != data.end(); datumToSend++)
		send(**datumToSend, delayTime);

	return true;
}

std::string TCPStream::receive() {
	const size_t messageSize = 256;
	char messageRecived[messageSize];

	int iResult = ::recv(socketDescriptor, messageRecived, messageSize, 0);
	if (iResult == 0 || iResult == SOCKET_ERROR)
		throw ExecuteWinSocketFailed("Recive Failed!", iResult);

	return std::string(messageRecived);
}