#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>

#include <vector>

#include "TCPExceptions.h"

using namespace std;

class message {
public:
	std::string getMessage() const;

protected:
	std::string messageToSend;
	virtual void makeMessage() = 0;

	friend class TCPStream;
};

class TCPStream {
	int		socketDescriptor;
	string	peerIP;
	int		peerPort;

	std::vector< message* > data;

public:
	~TCPStream();

	template< typename Container >
	const bool	addMessages(Container& dataToSend);

	const bool	send(message& datumToSend, const unsigned int delayTime = 5);
	const bool	send(const unsigned int delayTime = 1);

	std::string receive();
 
	string getPeerIP()	{ return peerIP; }
	int getPeerPort()	{ return peerPort; }

private:
	TCPStream();
	TCPStream(int socketdescriptor, SOCKADDR_IN* address);

	friend class TCPAcceptor;
	friend class TCPConnector;
};

template< typename Container >
const bool TCPStream::addMessages(Container& dataToSend) {
	data.clear();
	data.resize(0);

	for (auto elementIter = dataToSend.begin(); elementIter != dataToSend.end(); elementIter++)
		data.push_back(&*elementIter);

	return true;
}