#include <rsRobots/Robot>

using namespace rsRobots;

Robot::Robot(short form) {
	_form = form;
	_id = -1;
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
float* Robot::getRGB(void) {
	return _rgb;
}

const rs::Pos Robot::getRobotBodyPosition(short body, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// calculate offset position
	return P.add(q.multiply(_offset[body].x(), _offset[body].y(), _offset[body].z()));
}

void Robot::setRGB(const rs::Vec &c) {
	_rgb[0] = c[0];
	_rgb[1] = c[1];
	_rgb[2] = c[2];
}

