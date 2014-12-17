#include <rsRobots/robot.hpp>

using namespace rsRobots;

Robot::Robot(int form) {
	_form = form;
	_id = 0;
	_rgb[0] = 0;
	_rgb[1] = 1;
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

double* Robot::getRGB(void) {
	return _rgb;
}

void Robot::setTrace(bool trace) {
	_trace = trace;
}

