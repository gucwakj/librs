#include <cstdarg>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Requester>

using namespace rsResearch;

Requester::Requester(std::string ip, short port, short id) {
	// save data
	_ip = ip;
	_port = port;

	// add first message protocol
	this->addMessage(id);
}

Requester::~Requester(void) {
	for (unsigned int i = 0; i < _socket.size(); i++) {
		zmq_close(_socket[i]);
		zmq_ctx_destroy(_context[i]);
	}
}

/**********************************************************
	public functions
 **********************************************************/
short Requester::addMessage(short id) {
	// setup zmq
	_context.push_back(zmq_ctx_new());
	_socket.push_back(zmq_socket(_context[id], ZMQ_REQ));

	// set protocol
	std::string protocol("tcp://");
	protocol.append(_ip);
	protocol.append(":");
	protocol.append(std::to_string(_port + id));

	// connect to socket
 	int rc = zmq_connect(_socket[id], protocol.c_str());
 	if (rc) std::cerr << "rsResearch::Requester cannot connect to socket" << std::endl;

	// done
	return rc;
}

short Requester::request(short id, const char *format, ...) {
	// send request
	char outbuffer[256];
	sprintf(outbuffer, "%d", id);
	zmq_send(_socket[id], outbuffer, strlen(outbuffer), 0);

	// get response
	char inbuffer[256];
	zmq_recv(_socket[id], inbuffer, 255, 0);

	// create, scan, close va list
	va_list args;
	va_start(args, format); 
	vsscanf(inbuffer, format, args);
	va_end(args);

	// done
	return 0;
}

