#include <rsRobots/Robot>

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

const rs::Pos Robot::getRobotBodyPosition(int body, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// calculate offset position
	return P.add(q.multiply(_offset[body].x(), _offset[body].y(), _offset[body].z()));
}

void Robot::setID(int id) {
	_id = id;
}

void Robot::setForm(int form) {
	_form = form;
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setTrace(bool trace) {
	_trace = trace;
}

