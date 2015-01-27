#include <rsRobots/linkbot.hpp>

using namespace rsRobots;

LinkbotT::LinkbotT(int disabled) : Robot(rs::LINKBOTT) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_disabled = disabled;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_conn_depth = 0.00570;
	_conn_height = 0.03715;
	_bigwheel_radius = 0.05080;
	_bridge_length = 0.14025;
	_cubic_length = 0.07115;
	_omni_length = 0.17360;
	_smallwheel_radius = 0.04445;
	_tinywheel_radius = 0.04128;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;
	_offset.push_back(rs::Vec3(0, 0, 0));									// body
	_offset.push_back(rs::Vec3(-_body_width/2 - _face_depth/2, 0, 0));		// face1
	_offset.push_back(rs::Vec3(0, -_body_length - _face_depth/2, 0));		// face2
	_offset.push_back(rs::Vec3(_body_width/2 + _face_depth/2, 0, 0));		// face3
}

/**********************************************************
	public functions
 **********************************************************/
void LinkbotT::getConnFaceOffset(int type, int side, const double *p, const double *q, double *p1, double *q1) {
	// offsets
	double p2[3] = {0};
	double q2[4] = {0, 0, 0, 1};

	// get offset of face
	switch (type) {
		case rs::BRIDGE:
			p2[1] = -_bridge_length + 2*_face_radius;
			q2[0] = 0;
			q2[1] = sin(1.570796);	// 0.5*PI
			q2[2] = 0;
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
		case rs::CUBE:
			if (side == 2) {
				p2[0] = _cubic_length/2;
				p2[1] = _cubic_length/2;
				q2[0] = 0;
				q2[1] = 0;
				q2[2] = sin(0.785398);	// 0.5*PI/2
				q2[3] = cos(0.785398);	// 0.5*PI/2
			}
			else if (side == 3) {
				p2[0] = _cubic_length;
				q2[0] = 0;
				q2[1] = 0;
				q2[2] = 0;
				q2[3] = 1;
			}
			else if (side == 4) {
				p2[0] = _cubic_length/2;
				p2[1] = -_cubic_length/2;
				q2[0] = 0;
				q2[1] = 0;
				q2[2] = sin(-0.785398);	// -0.5*PI/2
				q2[3] = cos(-0.785398);	// -0.5*PI/2
			}
			else if (side == 5) {
				p2[0] = _cubic_length/2;
				p2[2] = _cubic_length/2;
				q2[0] = sin(-0.785398);	// -0.5*PI/2
				q2[1] = 0;
				q2[2] = sin(-0.785398);	// -0.5*PI/2
				q2[3] = cos(-0.785398);	// -0.5*PI/2
			}
			break;
		case rs::OMNIDRIVE:
			if (side == 2) {
				p2[2] = -_omni_length + 2*_face_radius;
			}
			else if (side == 3) {
				p2[1] = +_omni_length - 2*_face_radius;
			}
			else if (side == 4) {
				p2[1] = _omni_length - 2*_face_radius;
				p2[2] = -_omni_length + 2*_face_radius;
			}
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(1.570796);	// 0.5*PI
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
		case rs::SIMPLE:
			p2[0] = _conn_depth;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
			break;
	}

	// calculate offset position
	double v[3] = {p2[0], p2[1], p2[2]};
	double uv[3] = {q[1]*v[2]-q[2]*v[1], q[2]*v[0]-q[0]*v[2], q[0]*v[1]-q[1]*v[0]};
	double uuv[3] = {2*(q[1]*uv[2]-q[2]*uv[1]), 2*(q[2]*uv[0]-q[0]*uv[2]), 2*(q[0]*uv[1]-q[1]*uv[0])};
	p1[0] = p[0] + v[0] + 2*q[3]*uv[0] + uuv[0];
	p1[1] = p[1] + v[1] + 2*q[3]*uv[1] + uuv[1];
	p1[2] = p[2] + v[2] + 2*q[3]*uv[2] + uuv[2];

	// calculate offset quaternion
	q1[0] = q2[3]*q[0] + q2[0]*q[3] + q2[1]*q[2] - q2[2]*q[1];
	q1[1] = q2[3]*q[1] - q2[0]*q[2] + q2[1]*q[3] + q2[2]*q[0];
	q1[2] = q2[3]*q[2] + q2[0]*q[1] - q2[1]*q[0] + q2[2]*q[3];
	q1[3] = q2[3]*q[3] - q2[0]*q[0] - q2[1]*q[1] - q2[2]*q[2];
}

void LinkbotT::getConnBodyOffset(int type, const double *p, const double *q, double *p1, double *q1) {
	// offsets
	double p2[3] = {0};
	double q2[4] = {0, 0, 0, 1};

	// get offset of body
	switch (type) {
		case rs::BIGWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rs::BRIDGE:
			p2[0] = _conn_depth/2;
			p2[1] = -_bridge_length/2 + _face_radius;
			break;
		case rs::CASTER:
			p2[0] = _conn_depth/4;
			break;
		case rs::CUBE:
			p2[0] = _cubic_length/2;
			break;
		case rs::FACEPLATE:
			p2[0] = _conn_depth/2;
			break;
		case rs::GRIPPER:
			p2[0] = _conn_depth/2;
			break;
		case rs::OMNIDRIVE:
			p2[0] = _conn_depth/2;
			p2[1] = _omni_length/2 - _face_radius;
			p2[2] = -_omni_length/2 + _face_radius;
			break;
		case rs::SIMPLE:
			p2[0] = _conn_depth/2;
			break;
		case rs::SMALLWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rs::TINYWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rs::WHEEL:
			p2[0] = _wheel_depth/2;
			break;
	}

	// calculate offset position
	double v[3] = {p2[0], p2[1], p2[2]};
	double uv[3] = {q[1]*v[2]-q[2]*v[1], q[2]*v[0]-q[0]*v[2], q[0]*v[1]-q[1]*v[0]};
	double uuv[3] = {2*(q[1]*uv[2]-q[2]*uv[1]), 2*(q[2]*uv[0]-q[0]*uv[2]), 2*(q[0]*uv[1]-q[1]*uv[0])};
	p1[0] = p[0] + v[0] + 2*q[3]*uv[0] + uuv[0];
	p1[1] = p[1] + v[1] + 2*q[3]*uv[1] + uuv[1];
	p1[2] = p[2] + v[2] + 2*q[3]*uv[2] + uuv[2];

	// calculate offset quaternion
	q1[0] = q2[3]*q[0] + q2[0]*q[3] + q2[1]*q[2] - q2[2]*q[1];
	q1[1] = q2[3]*q[1] - q2[0]*q[2] + q2[1]*q[3] + q2[2]*q[0];
	q1[2] = q2[3]*q[2] + q2[0]*q[1] - q2[1]*q[0] + q2[2]*q[3];
	q1[3] = q2[3]*q[3] - q2[0]*q[0] - q2[1]*q[1] - q2[2]*q[2];
}

void LinkbotT::getRobotBodyOffset(int body, const double *p, const double *q, double *p1, double *q1) {
	// offset quaternion
	double q2[4] = {0, 0, 0, 1};
	switch (body) {
		case FACE1:
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(1.570796);	// 0.5 * PI
			q2[3] = cos(1.570796);	// 0.5 * PI
			break;
		case FACE2:
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(-0.785398);	// -0.5 * PI/2
			q2[3] = cos(-0.785398);	// -0.5 * PI/2
			break;
		case FACE3:
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
			break;
	}

	// calculate offset position
	double v[3] = {_offset[body].x, _offset[body].y, _offset[body].z};
	double uv[3] = {q[1]*v[2]-q[2]*v[1], q[2]*v[0]-q[0]*v[2], q[0]*v[1]-q[1]*v[0]};
	double uuv[3] = {2*(q[1]*uv[2]-q[2]*uv[1]), 2*(q[2]*uv[0]-q[0]*uv[2]), 2*(q[0]*uv[1]-q[1]*uv[0])};
	p1[0] = p[0] + v[0] + 2*q[3]*uv[0] + uuv[0];
	p1[1] = p[1] + v[1] + 2*q[3]*uv[1] + uuv[1];
	p1[2] = p[2] + v[2] + 2*q[3]*uv[2] + uuv[2];

	// calculate offset quaternion
	q1[0] = q2[3]*q[0] + q2[0]*q[3] + q2[1]*q[2] - q2[2]*q[1];
	q1[1] = q2[3]*q[1] - q2[0]*q[2] + q2[1]*q[3] + q2[2]*q[0];
	q1[2] = q2[3]*q[2] + q2[0]*q[1] - q2[1]*q[0] + q2[2]*q[3];
	q1[3] = q2[3]*q[3] - q2[0]*q[0] - q2[1]*q[1] - q2[2]*q[2];
}

void LinkbotT::getRobotFaceOffset(int face, const double *p, const double *q, double *p1, double *q1) {
	// get offset of face
	double p2[3] = {0};
	double q2[4];
	switch (face) {
		case FACE1:
			p2[0] = _face_depth/2;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(1.570796);	// 0.5*PI
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
		case FACE2:
			p2[1] = _face_depth/2;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(-0.785398);	// 0.5*PI/2
			q2[3] = cos(-0.785398);	// 0.5*PI/2
			break;
		case FACE3:
			p2[0] = _face_depth/2;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
			break;
	}

	// calculate offset position
	double v[3] = {_offset[face].x + p2[0], _offset[face].y + p2[1], _offset[face].z + p2[2]};
	double uv[3] = {q[1]*v[2]-q[2]*v[1], q[2]*v[0]-q[0]*v[2], q[0]*v[1]-q[1]*v[0]};
	double uuv[3] = {2*(q[1]*uv[2]-q[2]*uv[1]), 2*(q[2]*uv[0]-q[0]*uv[2]), 2*(q[0]*uv[1]-q[1]*uv[0])};
	p1[0] = p[0] + v[0] + 2*q[3]*uv[0] + uuv[0];
	p1[1] = p[1] + v[1] + 2*q[3]*uv[1] + uuv[1];
	p1[2] = p[2] + v[2] + 2*q[3]*uv[2] + uuv[2];

	// calculate offset quaternion
	q1[0] = q2[3]*q[0] + q2[0]*q[3] + q2[1]*q[2] - q2[2]*q[1];
	q1[1] = q2[3]*q[1] - q2[0]*q[2] + q2[1]*q[3] + q2[2]*q[0];
	q1[2] = q2[3]*q[2] + q2[0]*q[1] - q2[1]*q[0] + q2[2]*q[3];
	q1[3] = q2[3]*q[3] - q2[0]*q[0] - q2[1]*q[1] - q2[2]*q[2];
}

