#include <iostream>
#include <rsXML/Conn>

using namespace rsXML;

Conn::Conn(void) {
	_conn = 0;
	_face1 = 0;
	_face2 = 0;
	_robot = 0;
	_side = 0;
	_size = 0;
	_type = 0;
}

Conn::Conn(double size, int conn, int face1, int face2, int robot, int side, int type) {
	_conn = conn;
	_face1 = face1;
	_face2 = face2;
	_robot = robot;
	_side = side;
	_size = size;
	_type = type;
}

double Conn::getSize(void) {
	return _size;
}

int Conn::getConn(void) {
	return _conn;
}

int Conn::getFace1(void) {
	return _face1;
}

int Conn::getFace2(void) {
	return _face2;
}

int Conn::getRobot(void) {
	return _robot;
}

int Conn::getSide(void) {
	return _side;
}

int Conn::getType(void) {
	return _type;
}

void Conn::printDebug(void) {
	std::cerr << "XML Conn" << std::endl;
	std::cerr << "face: " << _face2 << std::endl;
	std::cerr << "base: " << _robot << std::endl;
	std::cerr << " fac: " << _face1 << std::endl;
	std::cerr << "type: " << _type << std::endl;
	std::cerr << "side: " << _side << std::endl;
	std::cerr << "conn: " << _conn << std::endl;
}
