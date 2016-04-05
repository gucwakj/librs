#include <cstdarg>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Replier>

using namespace rsResearch;

Replier::Replier(short port, short id) {
	// save port
	_port = port;

	// add first message
	this->addMessage(id);
}

Replier::~Replier(void) {
	for (unsigned int i = 0; i < _socket.size(); i++) {
		zmq_close(_socket[i]);
		zmq_ctx_destroy(_context[i]);
	}
}

/**********************************************************
	public functions
 **********************************************************/
short Replier::addMessage(short id) {
	// create zmq context
	_context.push_back(zmq_ctx_new());
	_socket.push_back(zmq_socket(_context[id], ZMQ_REP));

	// set protocol
	std::string protocol("tcp://*:");
	protocol.append(std::to_string(_port + id));

	// bind to socket
 	int rc = zmq_bind(_socket[id], protocol.c_str());
 	if (rc) std::cerr << "rsResearch::Replier cannot bind to socket" << std::endl;

	// done
	return rc;
}

short Replier::reply(short id, const char *format, ...) {
	// get request
	char inbuffer[256];
	zmq_recv(_socket[id], inbuffer, 255, 0);

	// buffer
	char outbuffer[256];

	// create, read, close va list
	va_list args;
	va_start(args, format); 
	vsnprintf(outbuffer, 255, format, args);
	outbuffer[strlen(outbuffer)] = '\0';
	va_end(args);

	// send message
	return zmq_send(_socket[id], outbuffer, strlen(outbuffer), 0);
}

