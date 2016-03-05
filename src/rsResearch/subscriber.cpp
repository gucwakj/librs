#include <cstdarg>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Subscriber>

using namespace rsResearch;

Subscriber::Subscriber(std::string ip, std::string prefix, std::string name, short port) {
	// save data
	_ip = ip;
	_port = port;

	// set prefix
	_prefix = prefix;
	_prefix.append(": ");
	if (name.size() != 0) {
		_prefix.append(name);
		_prefix.append(" ");
	}

	// add first message protocol
	this->addMessage(0);
}

Subscriber::~Subscriber(void) {
	for (unsigned int i = 0; i < _socket.size(); i++) {
		zmq_close(_socket[i]);
		zmq_ctx_destroy(_context[i]);
	}
}

/**********************************************************
	public functions
 **********************************************************/
short Subscriber::addMessage(short id) {
	// setup zmq
	_context.push_back(zmq_ctx_new());
	_socket.push_back(zmq_socket(_context[id], ZMQ_SUB));

	// add subscription to socket
	short rc = this->addSubscription(_ip, _port + id, id);

	// set options
	zmq_setsockopt(_socket[id], ZMQ_SUBSCRIBE, _prefix.c_str(), _prefix.size());

	return rc;
}

short Subscriber::addSubscription(std::string ip, short port, short id) {
	// set protocol
	std::string protocol("tcp://");
	protocol.append(ip);
	protocol.append(":");
	protocol.append(std::to_string(port));

	// connect to socket
 	int rc = zmq_connect(_socket[id], protocol.c_str());

 	if (rc) std::cerr << "rsResearch::Subscriber cannot connect to socket" << std::endl;
	return rc;
}

short Subscriber::receive(short id, const char *format, ...) {
	// get string
	char string[256];
	int size = this->receiveRaw(string, 256, id);

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

short Subscriber::receiveRaw(char *string, short length, short id) {
	// receive
	int size = zmq_recv(_socket[id], string, length - 1, ZMQ_DONTWAIT);

	// terminate string
	string[size] = '\0';

	// return
	return size;
}

