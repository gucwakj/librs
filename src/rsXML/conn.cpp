#include <rsXML/conn.hpp>

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

