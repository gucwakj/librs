#include <chrono>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Subscriber>

using namespace rsResearch;

Subscriber::Subscriber(std::string ip, int port, int id) {
	// robot id
	_id = id;

	// setup zmq
	_context = zmq_ctx_new();
	_socket = zmq_socket(_context, ZMQ_SUB);

	// set protocol
	std::string protocol("tcp://");
	protocol.append(ip);
	protocol.append(":");
	protocol.append(std::to_string(port));

	// connect to socket
 	if (zmq_connect(_socket, protocol.c_str()))
		std::cerr << "rsResearch::Subscriber cannot connect to socket" << std::endl;

	// set options
	std::string sub("robot");
	char str[4];
	std::sprintf(str, "%02d", id);
	sub.append(str);
	sub.append(": ");
	_filter_size = sub.size();
	zmq_setsockopt(_socket, ZMQ_SUBSCRIBE, sub.c_str(), _filter_size);
}

Subscriber::~Subscriber(void) {
	zmq_close(_socket);
	zmq_ctx_destroy(_context);
}

/**********************************************************
	public member functions
 **********************************************************/
double Subscriber::getAngle(void) {
	return _angle;
}

unsigned int Subscriber::getTime(void) {
	return _time;
}

void Subscriber::receive(void) {
	// get string
	char string[256];
	int size = zmq_recv(_socket, string, 255, 0);
	string[size] = 0;

	// read from string
	_angle = 0;
	_time = 0;
	sscanf(&string[_filter_size], "%u %lf", &_time, &_angle);
}

