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
int Robot::getOffset(int body, double *p) {
	p[0] = _offset[body].x;
	p[1] = _offset[body].y;
	p[2] = _offset[body].z;

	return 0;
}

int Robot::getRGB(double *rgb) {
	rgb = _rgb;
}

int Robot::setTrace(int trace) {
	_trace = trace;
}

