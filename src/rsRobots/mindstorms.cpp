#include <rsRobots/Mindstorms>

using namespace rsRobots;
using namespace rsMindstorms;

Mindstorms::Mindstorms(int form) : Robot(form) {
	_bigwheel_radius = 0.04100;
	_body_height = 0.123879;
	_body_length = 0.148832;
	_body_width = 0.087319;
	_smallwheel_radius = 0.02800;
	_wheel_depth = 0.02660;
	_wheel_radius = 0.02800;
	_offset.push_back(rs::Pos(0, -0.03568, 0.06700));						// body
	_offset.push_back(rs::Pos(-_body_width/2 - _wheel_depth/2, 0, 0));		// wheel1
	_offset.push_back(rs::Pos(_body_width/2 + _wheel_depth/2, 0, 0));		// wheel2
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Quat Mindstorms::getRobotBodyQuaternion(int body, double theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// wheel rotation
	Q.multiply(rs::Quat(sin(0.5*theta), 0, 0, cos(0.5*theta)));

	// return offset quaternion
	return Q;
}

double Mindstorms::getWheelRatio(int type) {
	switch (type) {
		case rsMindstorms::SMALL:
			return 1.0;
		case rsMindstorms::BIG:
			return _bigwheel_radius/_smallwheel_radius;
	}
	return 0;
}

double Mindstorms::riseByWheels(int type) {
	if (type == SMALL)
		return _smallwheel_radius;
	else if (type == BIG)
		return _bigwheel_radius;
	return -1;
}

const rs::Quat Mindstorms::tiltForWheels(int type1, int type2, double &p2) {
	// set wheel on this face
	_wheels[JOINT1] = type1;
	_wheels[JOINT2] = type2;

	// tilt
	if (type1 == NONE && type2 == NONE) {
		p2 = 0.021420;
		return rs::Quat(-0.0746256, 0, 0, 0.997212);
	}
	else if (type1 == SMALL && type2 == SMALL) {
		p2 = 0.028812;
		return rs::Quat(-0.022158, 0, 0, 0.999755);
	}
	else if (type1 == SMALL && type2 == BIG) {
		p2 = 0.034112;
		return rs::Quat(0.027590, -0.055976, 0.033117, 0.997501);
	}
	else if (type1 == BIG && type2 == SMALL) {
		p2 = 0.034112;
		return rs::Quat(0.027590, 0.055976, -0.033117, 0.997501);
	}
	else if (type1 == BIG && type2 == BIG) {
		p2 = 0.037561;
		return rs::Quat(0.048244, 0, 0, 0.998836);
	}

	// return default
	p2 = 0;
	return rs::Quat();
}

