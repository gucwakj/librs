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
void Mindstorms::getRobotBodyPosition(int body, double theta, const double *p, const rs::Quat &q, double *p1) {
	// calculate offset position
	double o[3];
	q.multiply(_offset[body].x, _offset[body].y, _offset[body].z, o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];
}

const rs::Quat Mindstorms::getRobotBodyQuaternion(int body, double theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// offset quaternion
	if (body == WHEEL1)
		Q.multiply(rs::Quat(0, 0, sin(1.570796), cos(1.570796)));

	// wheel rotation
	Q.multiply(rs::Quat(sin(0.5*theta), 0, 0, cos(0.5*theta)));

	// return offset quaternion
	return Q;
}

