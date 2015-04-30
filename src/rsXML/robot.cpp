#include <iostream>

#include <rs/Macros>
#include <rsXML/Robot>

using namespace rsXML;

Robot::Robot(bool trace) : rsRobots::Robot(rs::ROBOT) {
	_connected = 0;
	_c.allocate(4);
	_ground = -1;
	_id = -1;
	_trace = trace;
	_wheels.allocate(2);
}

Robot::~Robot(void) {
	for (unsigned int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int Robot::addConnector(Conn *conn) {
	_conn.push_back(conn);

	// success
	return 0;
}

Conn* Robot::getBaseConnector(void) {
	return _base;
}

ConnectorList& Robot::getConnectorList(void) {
	return _conn;
}

int Robot::getConnect(void) {
	return _connected;
}

int Robot::getGround(void) {
	return _ground;
}

int Robot::getID(void) {
	return _id;
}

const rs::Vec Robot::getJoints(void) {
	return _a;
}

const rs::Vec Robot::getLED(void) {
	return _c;
}

std::string Robot::getName(void) {
	return _name;
}

int Robot::getOrientation(void) {
	return _orientation;
}

const rs::Pos Robot::getPosition(void) {
	return _p;
}

const rs::Quat Robot::getQuaternion(void) {
	return _q;
}

double Robot::getRadius(void) {
	return _wheel_radius;
}

bool Robot::getTrace(void) {
	return _trace;
}

const rs::Vec Robot::getWheels(void) {
	return _wheels;
}

void Robot::printDebug(void) {
	std::cerr << "XML Robot" << std::endl;
	std::cerr << "form: " << _form << std::endl;
	std::cerr << "  id: " << _id << std::endl;
	_p.print();
	_q.print();
	_a.print();
	_c.print();
	for (unsigned int i = 0; i < _conn.size(); i++) {
		_conn[i]->printDebug();
	}
}

void Robot::setConnect(int a) {
	_connected = a;
}

void Robot::setGround(int a) {
	_ground = a;
}

void Robot::setID(int a) {
	_id = a;
}

void Robot::setJoints(double a, double b) {
	_a.allocate(2);
	_a[0] = a;
	_a[1] = b;
}

void Robot::setJoints(double a, double b, double c) {
	_a.allocate(3);
	_a[0] = a;
	_a[1] = b;
	_a[2] = c;
}

void Robot::setLED(double a, double b, double c, double d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setOrientation(int a) {
	_orientation = a;
}

void Robot::setPsi(double c) {
	_q.multiply(0, 0, sin(0.5*c), cos(0.5*c));
}

void Robot::setPosition(double a, double b, double c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Robot::setRotation(double a, double b, double c) {
	_q.multiply(sin(0.5*a), 0, 0, cos(0.5*a));
	_q.multiply(0, sin(0.5*b), 0, cos(0.5*b));
	_q.multiply(0, 0, sin(0.5*c), cos(0.5*c));
}

void Robot::setWheels(int left, int right) {
	_wheels[0] = left;
	_wheels[1] = right;
}

