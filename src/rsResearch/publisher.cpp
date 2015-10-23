#include <cstring>
#include <iostream>
#include <string>

#include <zmq.h>

#include <rsResearch/Publisher>

using namespace rsResearch;

Publisher::Publisher(int port) {
	_context = zmq_ctx_new();
	_socket = zmq_socket(_context, ZMQ_PUB);
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
int Publisher::send(short id, unsigned int time, float angle) {
	char buffer[256];
	sprintf(buffer, "hard%02d: %u %f", id, time, angle);
	return zmq_send(_socket, buffer, strlen(buffer), 0);
}

int Publisher::send(short id, float v, float r, float phi) {
	char buffer[256];
	sprintf(buffer, "cpg%02d: %f %f %f", id, v, r, phi);
	return zmq_send(_socket, buffer, strlen(buffer), 0);
}

