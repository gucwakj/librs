#include "robot.hpp"

using namespace rsXML;

Robot::Robot(void) {
	Robot(-1, 0);
}

Robot::Robot(int form, bool trace) {
	_a[0] = 0;
	_a[1] = 0;
	_a[2] = 0;
	_a[3] = 0;
	_a[4] = 0;
	_a[5] = 0;
	_connected = 0;
	_form = form;
	_ground = -1;
	_id = -1;
	_p[0] = 0;
	_p[1] = 0;
	_p[2] = 0;
	_q[0] = 0;
	_q[1] = 0;
	_q[2] = 0;
	_q[3] = 1;
	_trace = trace;
}

Robot::~Robot(void) {
	for (int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int Robot::addConnector(Conn *conn) {
	_conn.push_back(conn);

	// success
	return 0;
}

int Robot::getConnect(void) {
	return _connected;
}

int Robot::getForm(void) {
	return _form;
}

int Robot::getGround(void) {
	return _ground;
}

int Robot::getID(void) {
	return _id;
}

double* Robot::getJoints(void) {
	return _a;
}

double* Robot::getPosition(void) {
	return _p;
}

double* Robot::getQuaternion(void) {
	return _q;
}

bool Robot::getTrace(void) {
	return _trace;
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

void Robot::setJoints(double a, double b, double c, double d, double e, double f) {
	_a[0] = a;
	_a[1] = b;
	_a[2] = c;
	_a[3] = d;
	_a[4] = e;
	_a[5] = f;
}

void Robot::setPsi(double c) {
	// quaternions to multiply
	double q1[4] = {_q[0], _q[1], _q[2], _q[3]};
	double q2[4] = {0, 0, sin(0.5*c), cos(0.5*c)};

	// calculate new quaternion
	_q[0] = q2[3]*q1[0] + q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1];
	_q[1] = q2[3]*q1[1] - q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0];
	_q[2] = q2[3]*q1[2] + q2[0]*q1[1] - q2[1]*q1[0] + q2[2]*q1[3];
	_q[3] = q2[3]*q1[3] - q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2];
}

void Robot::setPosition(double a, double b, double c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Robot::setRotation(double a, double b, double c) {
	double q1[4] = {sin(0.5*a), 0, 0, cos(0.5*a)};
	double q2[4] = {0, sin(0.5*b), 0, cos(0.5*b)};
	double q3[4] = {0, 0, sin(0.5*c), cos(0.5*c)};

	_q[0] = q3[3]*q2[3]*q1[0] + q3[3]*q2[0]*q1[3] + q3[3]*q2[1]*q1[2] - q3[3]*q2[2]*q1[1] +
			q3[0]*q2[3]*q1[3] - q3[0]*q2[0]*q1[0] - q3[0]*q2[1]*q1[1] - q3[0]*q2[2]*q1[2] +
			q3[1]*q2[3]*q1[2] + q3[1]*q2[0]*q1[1] - q3[1]*q2[1]*q1[0] + q3[1]*q2[2]*q1[3] -
			q3[2]*q2[3]*q1[1] + q3[2]*q2[0]*q1[2] - q3[2]*q2[1]*q1[3] - q3[2]*q2[2]*q1[0];
	_q[1] = q3[3]*q2[3]*q1[1] - q3[3]*q2[0]*q1[2] + q3[3]*q2[1]*q1[3] + q3[3]*q2[2]*q1[0] -
			q3[0]*q2[3]*q1[2] - q3[0]*q2[0]*q1[1] + q3[0]*q2[1]*q1[0] - q3[0]*q2[2]*q1[3] +
			q3[1]*q2[3]*q1[3] - q3[1]*q2[0]*q1[0] - q3[1]*q2[1]*q1[1] - q3[1]*q2[2]*q1[2] +
			q3[2]*q2[3]*q1[0] + q3[2]*q2[0]*q1[3] + q3[2]*q2[1]*q1[2] - q3[2]*q2[2]*q1[1];
	_q[2] = q3[3]*q2[3]*q1[2] + q3[3]*q2[0]*q1[1] - q3[3]*q2[1]*q1[0] + q3[3]*q2[2]*q1[3] +
			q3[0]*q2[3]*q1[1] - q3[0]*q2[0]*q1[2] + q3[0]*q2[1]*q1[3] + q3[0]*q2[2]*q1[0] -
			q3[1]*q2[3]*q1[0] - q3[1]*q2[0]*q1[3] - q3[1]*q2[1]*q1[2] + q3[1]*q2[2]*q1[1] +
			q3[2]*q2[3]*q1[3] - q3[2]*q2[0]*q1[0] - q3[2]*q2[1]*q1[1] - q3[2]*q2[2]*q1[2];
	_q[3] = q3[3]*q2[3]*q1[3] - q3[3]*q2[0]*q1[0] - q3[3]*q2[1]*q1[1] - q3[3]*q2[2]*q1[2] -
			q3[0]*q2[3]*q1[0] - q3[0]*q2[0]*q1[3] - q3[0]*q2[1]*q1[2] + q3[0]*q2[2]*q1[1] -
			q3[1]*q2[3]*q1[1] + q3[1]*q2[0]*q1[2] - q3[1]*q2[1]*q1[3] - q3[1]*q2[2]*q1[0] -
			q3[2]*q2[3]*q1[2] - q3[2]*q2[0]*q1[1] + q3[2]*q2[1]*q1[0] - q3[2]*q2[2]*q1[3];
}

