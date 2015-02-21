#include <rsRobots/Mindstorms.hpp>

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
void Mindstorms::getRobotBodyOffset(int body, const double *p, const double *q, double *p1, double *q1) {
	// offset quaternion
	double q2[4] = {0, 0, 0, 1};
	switch (body) {
		case WHEEL1:
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(1.570796);	// 0.5 * PI
			q2[3] = cos(1.570796);	// 0.5 * PI
			break;
		case WHEEL2:
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
			break;
	}

	// calculate offset position
	double o[3];
	this->multiplyQbyV(q, _offset[body].x, _offset[body].y, _offset[body].z, o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];

	// calculate offset quaternion
	this->multiplyQbyQ(q, q2, q1);
}

