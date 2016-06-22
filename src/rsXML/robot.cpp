#include <iostream>

#include <rsXML/Robot>

using namespace rsXML;

Robot::Robot(bool trace) : rsRobots::Robot(rs::Robot) {
	_base = NULL;
	_connected = false;
	_c.allocate(4);
	_c[0] = 1;
	_c[1] = 0;
	_c[2] = 0;
	_c[3] = 1;
	_enabled = 1;
	_ground = -1;
	_id = -1;
	_orientation = 0;
	_shape = 0;
	_trace = trace;
	_wheels.allocate(2);
	_wheels[0] = 0;
	_wheels[1] = 0;
}

Robot::~Robot(void) {
	for (unsigned int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

/**********************************************************
	public functions
 **********************************************************/
short Robot::addConnector(Conn *conn) {
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

bool Robot::getConnect(void) {
	return _connected;
}

short Robot::getEnabled(void) {
	return _enabled;
}

short Robot::getGround(void) {
	return _ground;
}

short Robot::getID(void) {
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

short Robot::getOrientation(void) {
	return _orientation;
}

const rs::Pos Robot::getPosition(void) {
	return _p;
}

const rs::Quat Robot::getQuaternion(void) {
	return _q;
}

short Robot::getShape(void) {
	return _shape;
}

bool Robot::getTrace(void) {
	return _trace;
}

const rs::Vec Robot::getWheels(void) {
	return _wheels;
}

void Robot::printDebug(void) {
	std::cerr << "XML Robot" << std::endl;
	std::cerr << "form: " << this->getForm() << std::endl;
	std::cerr << "  id: " << this->getID() << std::endl;
	_p.print();
	_q.print();
	_a.print();
	_c.print();
	for (unsigned int i = 0; i < _conn.size(); i++) {
		_conn[i]->printDebug();
	}
}

void Robot::setConnect(bool b) {
	_connected = b;
}

void Robot::setEnabled(short a) {
	_enabled = a;
}

void Robot::setGround(short a) {
	_ground = a;
}

void Robot::setID(short a) {
	_id = a;
}

void Robot::setJoints(float a, float b) {
	if (_a.size() >= 1) _a[0] = a;
	if (_a.size() >= 2) _a[1] = b;
}

void Robot::setJoints(float a, float b, float c) {
	if (_a.size()>= 1) _a[0] = a;
	if (_a.size()>= 2) _a[1] = b;
	if (_a.size()>= 3) _a[2] = c;
}

void Robot::setLED(float a, float b, float c, float d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setOrientation(short a) {
	_orientation = a;
}

void Robot::setPsi(float c) {
	_q.multiply(0, 0, sin(0.5*c), cos(0.5*c));
}

void Robot::setPosition(float a, float b, float c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Robot::setRotation(float a, float b, float c) {
	_q.multiply(sin(0.5*a), 0, 0, cos(0.5*a));
	_q.multiply(0, sin(0.5*b), 0, cos(0.5*b));
	_q.multiply(0, 0, sin(0.5*c), cos(0.5*c));
}

void Robot::setRotation(float a, float b, float c, float d) {
	_q[0] = a;
	_q[1] = b;
	_q[2] = c;
	_q[3] = d;
}

void Robot::setWheels(short left, short right) {
	_wheels[0] = left;
	_wheels[1] = right;
}

