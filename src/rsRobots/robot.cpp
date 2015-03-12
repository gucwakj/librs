#include <rsRobots/Robot>
#include <iostream>

using namespace rsRobots;

Robot::Robot(int form) {
	_form = form;
	_id = 0;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 0;
	_trace = 0;
}

Robot::~Robot(void) {
	_offset.clear();
}

/**********************************************************
	public functions
 **********************************************************/
int Robot::getForm(void) {
	return _form;
}

int Robot::getID(void) {
	return _id;
}

std::string Robot::getName(void) {
	return _name;
}

double* Robot::getRGB(void) {
	return _rgb;
}

void Robot::multiplyQbyV(const double *q, double v1, double v2, double v3, double *o) {
	double uv[3] = {q[1]*v3 - q[2]*v2, q[2]*v1 - q[0]*v3, q[0]*v2 - q[1]*v1};
	double uuv[3] = {2*(q[1]*uv[2] - q[2]*uv[1]), 2*(q[2]*uv[0] - q[0]*uv[2]), 2*(q[0]*uv[1] - q[1]*uv[0])};
	o[0] = v1 + 2*q[3]*uv[0] + uuv[0];
	o[1] = v2 + 2*q[3]*uv[1] + uuv[1];
	o[2] = v3 + 2*q[3]*uv[2] + uuv[2];
}

void Robot::multiplyQbyQ(const double *q, const double *q2, double *o) {
	o[0] = q2[3]*q[0] + q2[0]*q[3] + q2[1]*q[2] - q2[2]*q[1];
	o[1] = q2[3]*q[1] - q2[0]*q[2] + q2[1]*q[3] + q2[2]*q[0];
	o[2] = q2[3]*q[2] + q2[0]*q[1] - q2[1]*q[0] + q2[2]*q[3];
	o[3] = q2[3]*q[3] - q2[0]*q[0] - q2[1]*q[1] - q2[2]*q[2];
}

void Robot::setID(int id) {
	_id = id;
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setTrace(bool trace) {
	_trace = trace;
}

