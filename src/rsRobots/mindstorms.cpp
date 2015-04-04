#include <rsRobots/Mindstorms>

using namespace rsRobots;
using namespace rsMindstorms;

Mindstorms::Mindstorms(int form) : Robot(form) {
	_body_height = 0.13400;
	_body_length = 0.17300;
	_body_width = 0.09175;
	_radius = 0.02843;
	_offset.push_back(rs::Pos(0, -0.03568, 0.06700));						// body
	_offset.push_back(rs::Pos(-_body_width/2 - _wheel_depth/2, 0, 0));		// wheel1
	_offset.push_back(rs::Pos(_body_width/2 + _wheel_depth/2, 0, 0));		// wheel2
	_wheel_depth = 0.02660;
	_wheel_radius = _radius;
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Quat Mindstorms::getRobotBodyQuaternion(int body, double theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// calculate offset quaternion
	if (body == WHEEL1)
		Q.multiply(rs::Quat(0, 0, sin(1.570796), cos(1.570796)));

	// wheel rotation
	Q.multiply(rs::Quat(sin(0.5*theta), 0, 0, cos(0.5*theta)));

	// return offset quaternion
	return Q;
}

