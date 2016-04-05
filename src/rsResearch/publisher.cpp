#include <cstdarg>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Publisher>

using namespace rsResearch;

Publisher::Publisher(short port, std::string prefix, short id) {
	// save port
	_port = port;

	// save message prefix
	_prefix = prefix;
	_prefix.append(":");

	// add first message
	this->addMessage(id);
}

Publisher::~Publisher(void) {
	for (unsigned int i = 0; i < _socket.size(); i++) {
		zmq_close(_socket[i]);
		zmq_ctx_destroy(_context[i]);
	}
}

/**********************************************************
	public functions
 **********************************************************/
void Publisher::addMessage(short id) {
	// create zmq context
	_context.push_back(zmq_ctx_new());
	_socket.push_back(zmq_socket(_context[id], ZMQ_PUB));

	// open port for sending
	std::string protocol("tcp://*:");
	protocol.append(std::to_string(_port + id));
	if (zmq_bind(_socket[id], protocol.c_str()))
		std::cerr << "rsResearch::Publisher cannot bind to socket" << std::endl;
}

short Publisher::send(short id, const char *format, ...) {
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
	return zmq_send(_socket[id], buffer, strlen(buffer), 0);
}

