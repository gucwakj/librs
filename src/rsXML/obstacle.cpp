#include <cmath>

#include <rsXML/Obstacle>

using namespace rsXML;

Obstacle::Obstacle(int type) {
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
int Obstacle::getAxis(void) {
	return _axis;
}

const rs::Vec Obstacle::getColor(void) {
	return _c;
}

int Obstacle::getConnect(void) {
	return _connected;
}

const rs::Vec Obstacle::getDimensions(void) {
	return _l;
}

int Obstacle::getForm(void) {
	return _type;
}

int Obstacle::getID(void) {
	return _id;
}

double Obstacle::getMass(void) {
	return _mass;
}

const rs::Pos Obstacle::getPosition(void) {
	return _p;
}

const rs::Quat Obstacle::getQuaternion(void) {
	return _q;
}

void Obstacle::setAxis(int i) {
	_axis = i;
}

void Obstacle::setColor(double a, double b, double c, double d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Obstacle::setConnect(int a) {
	_connected = a;
}

void Obstacle::setDimensions(double a, double b, double c) {
	if (fabs(a) > rs::Epsilon) _l[0] = a;
	if (fabs(b) > rs::Epsilon) _l[1] = b;
	if (fabs(c) > rs::Epsilon) _l[2] = c;
}

void Obstacle::setID(int a) {
	_id = a;
}

void Obstacle::setMass(double a) {
	_mass = a;
}

void Obstacle::setPosition(double a, double b, double c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Obstacle::setRotation(double a, double b, double c) {
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

