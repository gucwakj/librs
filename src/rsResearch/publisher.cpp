#include <cstdarg>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Publisher>

using namespace rsResearch;

Publisher::Publisher(int port, std::string prefix) {
	// save message prefix
	_prefix = prefix;
	_prefix.append(":");

	// create zmq context
	_context = zmq_ctx_new();
	_socket = zmq_socket(_context, ZMQ_PUB);

	// open port for sending
	std::string protocol("tcp://*:");
	protocol.append(std::to_string(port));
	if (zmq_bind(_socket, protocol.c_str()))
		std::cerr << "rsResearch::Publisher cannot bind to socket" << std::endl;
}

Publisher::~Publisher(void) {
	zmq_close(_socket);
	zmq_ctx_destroy(_context);
}

/**********************************************************
	public functions
 **********************************************************/
short Publisher::send(const char *format, ...) {
	// buffer
	char buffer[256];

	// put prefix into buffer
	int pre = _prefix.size() + 1;
	snprintf(buffer, pre, "%s", _prefix.c_str());
	buffer[_prefix.size()] = ' ';

	// create, read, close va list
	va_list args;
	va_start(args, format); 
	vsnprintf(&buffer[pre], 255 - pre, format, args);
	buffer[strlen(buffer)] = '\0';
	va_end(args);

	// send message
	return zmq_send(_socket, buffer, strlen(buffer), 0);
}

