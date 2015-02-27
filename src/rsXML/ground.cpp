#include <cmath>
#include <rsXML/Ground>

using namespace rsXML;

Ground::Ground(int type) {
	_axis = 0;
	_c[0] = 0;
	_c[1] = 0;
	_c[2] = 0;
	_c[3] = 0;
	_l[0] = 0;
	_l[1] = 0;
	_l[2] = 0;
	_mass = 0;
	_p[0] = 0;
	_p[1] = 0;
	_p[2] = 0;
	_q[0] = 0;
	_q[1] = 0;
	_q[2] = 0;
	_q[3] = 1;
	_type = type;
}

double Ground::getAxis(void) {
	return _axis;
}

double* Ground::getColor(void) {
	return _c;
}

double* Ground::getDimensions(void) {
	return _l;
}

double Ground::getMass(void) {
	return _mass;
}

double* Ground::getPosition(void) {
	return _p;
}

double* Ground::getQuaternion(void) {
	return _q;
}

int Ground::getType(void) {
	return _type;
}

void Ground::setAxis(double a) {
	_axis = a;
}

void Ground::setColor(double a, double b, double c, double d) {
	_c[0] = a;
	_c[1] = b;
	_c[2] = c;
	_c[3] = d;
}

void Ground::setDimensions(double a, double b, double c) {
	_l[0] = a;
	_l[1] = b;
	_l[2] = c;
}

void Ground::setMass(double a) {
	_mass = a;
}

void Ground::setPosition(double a, double b, double c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Ground::setRotation(double a, double b, double c) {
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

