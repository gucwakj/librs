#include <cmath>

#include <rsXML/Obstacle>

using namespace rsXML;

Obstacle::Obstacle(short type) {
	_axis = 1;
	_connected = 0;
	_mass = 0.1;
	_type = type;
	_l.allocate(3);
	_l[0] = 0.1;
	_l[1] = 0.1;
	_l[2] = 0.1;
	_c.allocate(4);
	_c[0] = 1;
	_c[1] = 0;
	_c[2] = 0;
	_c[3] = 1;
}

/**********************************************************
	public functions
 **********************************************************/
short Obstacle::getAxis(void) {
	return _axis;
}

const rs::Vec Obstacle::getColor(void) {
	return _c;
}

bool Obstacle::getConnect(void) {
	return _connected;
}

const rs::Vec Obstacle::getDimensions(void) {
	return _l;
}

short Obstacle::getForm(void) {
	return _type;
}

short Obstacle::getID(void) {
	return _id;
}

float Obstacle::getMass(void) {
	return _mass;
}

const rs::Pos Obstacle::getPosition(void) {
	return _p;
}

const rs::Quat Obstacle::getQuaternion(void) {
	return _q;
}

void Obstacle::setAxis(short i) {
	_axis = i;
}

void Obstacle::setColor(float a, float b, float c, float d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Obstacle::setConnect(bool b) {
	_connected = b;
}

void Obstacle::setDimensions(float a, float b, float c) {
	if (fabs(a) > rs::Epsilon) _l[0] = a;
	if (fabs(b) > rs::Epsilon) _l[1] = b;
	if (fabs(c) > rs::Epsilon) _l[2] = c;
}

void Obstacle::setID(int a) {
	_id = a;
}

void Obstacle::setMass(float a) {
	_mass = (fabs(a) < rs::Epsilon) ? 10000000 : a;
}

void Obstacle::setPosition(float a, float b, float c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Obstacle::setRotation(float a, float b, float c) {
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

void Obstacle::setRotation(float a, float b, float c, float d) {
	_q[0] = a;
	_q[1] = b;
	_q[2] = c;
	_q[3] = d;
}

