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
	public member functions
 **********************************************************/
int Publisher::send(int id, unsigned int time, double angle) {
	char buffer[256];
	sprintf(buffer, "robot%02d: %u %lf", id, time, angle);
	return zmq_send(_socket, buffer, strlen(buffer), 0);
}

