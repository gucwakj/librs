#include <rsRobots/Robot>

using namespace rsRobots;

Robot::Robot(short form) {
	_form = form;
	_id = 0;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 0;
	_trace = 0;
	_wheels[0] = -1;
	_wheels[1] = -1;
}

Robot::~Robot(void) {
	_offset.clear();
}

/**********************************************************
	public functions
 **********************************************************/
float Robot::getBodyHeight(void) {
	return _body_height;
}

short Robot::getForm(void) {
	return _form;
}

short Robot::getID(void) {
	return _id;
}

std::string Robot::getName(void) {
	return _name;
}

float* Robot::getRGB(void) {
	return _rgb;
}

const rs::Pos Robot::getRobotBodyPosition(short body, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// calculate offset position
	return P.add(q.multiply(_offset[body].x(), _offset[body].y(), _offset[body].z()));
}

bool Robot::getTrace(void) {
	return _trace;
}

float Robot::getWheelRadius(void) {
	return _wheel_radius;
}

void Robot::setID(short id) {
	_id = id;
}

void Robot::setForm(short form) {
	_form = form;
}

void Robot::setLED(const rs::Vec &c) {
	_rgb[0] = c[0];
	_rgb[1] = c[1];
	_rgb[2] = c[2];
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setTrace(bool trace) {
	_trace = trace;
}

void Robot::setWheelRadius(float radius) {
	_wheel_radius = radius;
}

