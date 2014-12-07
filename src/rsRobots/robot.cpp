#include <rsRobots/robot.hpp>

using namespace rsRobots;

Robot::Robot(int form) {
	_form = form;
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

void Robot::getOffsetPos(int body, const double *p, double *p1) {
	p1[0] = p[0] + _offset[body].x;
	p1[1] = p[1] + _offset[body].y;
	p1[2] = p[2] + _offset[body].z;
}

void Robot::getRGB(double *rgb) {
	rgb = _rgb;
}

void Robot::setTrace(bool trace) {
	_trace = trace;
}

