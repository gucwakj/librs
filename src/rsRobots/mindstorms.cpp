#include <rsRobots/Mindstorms>

using namespace rsRobots;
using namespace rsMindstorms;

Mindstorms::Mindstorms(void) : Robot(rs::MINDSTORMS) {
	_body_height = 0.07250;
	_body_length = 0.03935;
	_body_radius = 0.03625;
	_body_width = 0.07835;
	_radius = _body_height/2;
	_offset.push_back(rs::Vec3(0, 0, 0));									// body
	_offset.push_back(rs::Vec3(-_body_width/2 - _wheel_depth/2, 0, 0));		// wheel1
	_offset.push_back(rs::Vec3(_body_width/2 + _wheel_depth/2, 0, 0));		// wheel2
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;
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

