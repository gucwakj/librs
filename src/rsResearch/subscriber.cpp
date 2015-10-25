#include <chrono>
#include <cstring>
#include <iostream>

#include <zmq.h>

#include <rsResearch/Subscriber>

using namespace rsResearch;

Subscriber::Subscriber(std::string ip, std::string filter, short port, short id) {
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
	std::string sub(filter);
	char str[4];
	std::sprintf(str, "%02d", id);
	sub.append(str);
	sub.append(": ");
	_filter = filter.compare("hard") ? 0 : 1;
	_filter_size = sub.size();
	zmq_setsockopt(_socket, ZMQ_SUBSCRIBE, sub.c_str(), _filter_size);
}

Subscriber::~Subscriber(void) {
	zmq_close(_socket);
	zmq_ctx_destroy(_context);
}

/**********************************************************
	public functions
 **********************************************************/
float Subscriber::getAngle(void) {
	return _angle;
}

unsigned int Subscriber::getTime(void) {
	return _time;
}

float Subscriber::getV(void) {
	return _v;
}

float Subscriber::getR(void) {
	return _r;
}

float Subscriber::getPhi(void) {
	return _phi;
}

void Subscriber::receive(void) {
	// get string
	char string[256];
	int size = zmq_recv(_socket, string, 255, 0);
	string[size] = 0;

	// read from string
	if (_filter == 0) {
		_v = 0;
		_r = 0;
		_phi = 0;
		sscanf(&string[_filter_size], "%f %f %f", &_v, &_r, &_phi);
	}
	else if (_filter == 1) {
		_angle = 0;
		_time = 0;
		sscanf(&string[_filter_size], "%u %f", &_time, &_angle);
	}
}

