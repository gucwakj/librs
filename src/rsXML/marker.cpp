#include <rsXML/Marker>

using namespace rsXML;

Marker::Marker(short type) {
	_connected = false;
	_size = 1;
	_type = type;
	_c.allocate(4);
	_f.allocate(4);
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Vec Marker::getColor(void) {
	return _c;
}

bool Marker::getConnect(void) {
	return _connected;
}

const rs::Pos Marker::getEnd(void) {
	return _e;
}

const rs::Vec Marker::getFill(void) {
	return _f;
}

short Marker::getID(void) {
	return _id;
}

const rs::Pos Marker::getStart(void) {
	return _s;
}

short Marker::getSize(void) {
	return _size;
}

const rs::Pos Marker::getPoint(void) {
	return _p;
}

short Marker::getForm(void) {
	return _type;
}

std::string Marker::getLabel(void) {
	return _l;
}

void Marker::setColor(float a, float b, float c, float d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Marker::setConnect(bool b) {
	_connected = b;
}

void Marker::setEnd(float a, float b, float c) {
	_e[0] = a;
	_e[1] = b;
	_e[2] = c;
}

void Marker::setFill(float a, float b, float c, float d) {
	_f[0] = a;
	_f[1] = b;
	_f[2] = c;
	_f[3] = d;
}

void Marker::setID(short id) {
	_id = id;
}

void Marker::setLabel(std::string l) {
	_l = l;
}

void Marker::setPoint(float a, float b) {
	_p[0] = a;
	_p[1] = b;
}

void Marker::setSize(short size) {
	_size = size;
}

void Marker::setStart(float a, float b, float c) {
	_s[0] = a;
	_s[1] = b;
	_s[2] = c;
}

