#include <rsXML/Marker>

using namespace rsXML;

Marker::Marker(int type) {
	_c[0] = 0;
	_c[1] = 0;
	_c[2] = 0;
	_c[3] = 0;
	_connected = 0;
	_e[0] = 0;
	_e[1] = 0;
	_e[2] = 0;
	_s[0] = 0;
	_s[1] = 0;
	_s[2] = 0;
	_size = 1;
	_type = type;
}

double* Marker::getColor(void) {
	return _c;
}

int Marker::getConnect(void) {
	return _connected;
}

double* Marker::getEnd(void) {
	return _e;
}

int Marker::getID(void) {
	return _id;
}

double* Marker::getStart(void) {
	return _s;
}

int Marker::getSize(void) {
	return _size;
}

int Marker::getForm(void) {
	return _type;
}

std::string Marker::getLabel(void) {
	return _l;
}

void Marker::setColor(double a, double b, double c, double d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Marker::setConnect(int a) {
	_connected = a;
}

void Marker::setEnd(double a, double b, double c) {
	_e[0] = a;
	_e[1] = b;
	_e[2] = c;
}

void Marker::setID(int id) {
	_id = id;
}

void Marker::setLabel(std::string l) {
	_l = l;
}

void Marker::setSize(int size) {
	_size = size;
}

void Marker::setStart(double a, double b, double c) {
	_s[0] = a;
	_s[1] = b;
	_s[2] = c;
}

