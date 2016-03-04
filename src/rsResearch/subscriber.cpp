#include <cstdarg>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Subscriber>

using namespace rsResearch;

Subscriber::Subscriber(std::string ip, std::string filter, std::string id, short port) {
	// setup zmq
	_context = zmq_ctx_new();
	_socket = zmq_socket(_context, ZMQ_SUB);

	// add subscription to socket
	this->addSubscription(ip, port);

	// set options
	_prefix = filter;
	_prefix.append(": ");
	if (id.size() != 0) {
		_prefix.append(id);
		_prefix.append(" ");
	}
	zmq_setsockopt(_socket, ZMQ_SUBSCRIBE, _prefix.c_str(), _prefix.size());
}

Subscriber::~Subscriber(void) {
	zmq_close(_socket);
	zmq_ctx_destroy(_context);
}

/**********************************************************
	public functions
 **********************************************************/
short Subscriber::addSubscription(std::string ip, short port) {
	// set protocol
	std::string protocol("tcp://");
	protocol.append(ip);
	protocol.append(":");
	protocol.append(std::to_string(port));

	// connect to socket
 	int rc = zmq_connect(_socket, protocol.c_str());

 	if (rc) std::cerr << "rsResearch::Subscriber cannot connect to socket" << std::endl;
	return rc;
}

short Subscriber::receive(const char *format, ...) {
	// get string
	char string[256];
	int size = this->receiveRaw(string, 256);

	// no message right now, just return
	if (size == -1) {
		return -1;
	}

	// start, scan, end va list
	va_list args;
	va_start(args, format);
	vsscanf(&string[_prefix.size()], format, args);
	va_end(args);

	// done
	return 0;
}

short Subscriber::receiveRaw(char *string, short length) {
	// receive
	int size = zmq_recv(_socket, string, length - 1, ZMQ_DONTWAIT);

	// terminate string
	string[size] = '\0';

	// return
	return size;
}

