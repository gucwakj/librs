#include <iostream>
#include <rsRobots/Linkbot>

using namespace rsRobots;
using namespace rsLinkbot;

Linkbot::Linkbot(int disabled) : Robot(rs::LINKBOTT) {
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
void Linkbot::getConnFaceOffset(int type, int side, int orientation, const double *p, const double *q, double *p1, double *q1) {
	// rotate for orientation
	double q3[4] = {sin(0.5*1.570796*(orientation-1)),	// 0.5*90
					0, 0,
					cos(0.5*1.570796*(orientation-1))};	// 0.5*90
	double q4[4] = {0, 0, 0, 1};
	this->multiplyQbyQ(q, q3, q4);

	// offsets
	double p2[3] = {0};
	double q2[4] = {0, 0, 0, 1};

	// get offset of face
	switch (type) {
		case rsLinkbot::BRIDGE:
			p2[1] = -_bridge_length + 2*_face_radius;
			q2[0] = 0;
			q2[1] = sin(1.570796);	// 0.5*PI
			q2[2] = 0;
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
		case rsLinkbot::CUBE:
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
		case rsLinkbot::DOUBLEBRIDGE:
			if (side == 2) {
				p2[1] = -_bridge_length + 2*_face_radius;
				q2[0] = 0;
				q2[1] = sin(1.570796);	// 0.5*PI
				q2[2] = 0;
				q2[3] = cos(1.570796);	// 0.5*PI
			}
			else if (side == 3) {
				p2[0] = 2*_conn_depth;
			}
			else if (side == 4) {
				p2[0] = 2*_conn_depth;
				p2[1] = -_bridge_length + 2*_face_radius;
			}
			break;
		case rsLinkbot::OMNIPLATE:
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
		case rsLinkbot::SIMPLE:
			p2[0] = _conn_depth;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = 1;
			break;
	}

	// calculate offset position
	double o[3];
	this->multiplyQbyV(q4, p2[0], p2[1], p2[2], o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];

	// calculate offset quaternion
	this->multiplyQbyQ(q4, q2, q1);
}

void Linkbot::getConnBodyOffset(int type, int orientation, const double *p, const double *q, double *p1, double *q1) {
	// rotate for orientation
	double q3[4] = {sin(0.5*1.570796*(orientation-1)),	// 0.5*90
					0, 0,
					cos(0.5*1.570796*(orientation-1))};	// 0.5*90
	double q4[4] = {0, 0, 0, 1};
	this->multiplyQbyQ(q, q3, q4);

	// offsets
	double p2[3] = {0};
	double q2[4] = {0, 0, 0, 1};

	// get offset of body
	switch (type) {
		case rsLinkbot::BIGWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rsLinkbot::BRIDGE:
			p2[0] = _conn_depth/2;
			p2[1] = _bridge_length/2 - _face_radius;
			break;
		case rsLinkbot::CASTER:
			p2[0] = _conn_depth/4;
			break;
		case rsLinkbot::CUBE:
			p2[0] = _cubic_length/2;
			break;
		case rsLinkbot::DOUBLEBRIDGE:
			p2[0] = _conn_depth;
			p2[1] = _bridge_length/2 - _face_radius;
			break;
		case rsLinkbot::FACEPLATE:
			p2[0] = _conn_depth/2;
			break;
		case rsLinkbot::GRIPPER:
			p2[0] = _conn_depth/2;
			break;
		case rsLinkbot::OMNIPLATE:
			p2[0] = _conn_depth/2;
			p2[1] = _omni_length/2 - _face_radius;
			p2[2] = -_omni_length/2 + _face_radius;
			break;
		case rsLinkbot::SIMPLE:
			p2[0] = _conn_depth/2;
			break;
		case rsLinkbot::SMALLWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rsLinkbot::TINYWHEEL:
			p2[0] = _wheel_depth/2;
			break;
		case rsLinkbot::WHEEL:
			p2[0] = _wheel_depth/2;
			break;
	}

	// calculate offset position
	double o[3];
	this->multiplyQbyV(q4, p2[0], p2[1], p2[2], o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];

	// calculate offset quaternion
	this->multiplyQbyQ(q4, q2, q1);
}

void Linkbot::getRobotBodyOffset(int body, const double *p, const double *q, double *p1, double *q1) {
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
	double o[3];
	this->multiplyQbyV(q, _offset[body].x, _offset[body].y, _offset[body].z, o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];

	// calculate offset quaternion
	this->multiplyQbyQ(q, q2, q1);
}

void Linkbot::getRobotFaceOffset(int face, const double *p, const double *q, double *p1, double *q1) {
	// get offset of face
	double p2[3] = {0};
	double q2[4];
	switch (face) {
		case FACE1:
			p2[0] = -_face_depth/2;
			q2[0] = 0;
			q2[1] = 0;
			q2[2] = sin(1.570796);	// 0.5*PI
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
		case FACE2:
			p2[1] = -_face_depth/2;
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
	double o[3];
	this->multiplyQbyV(q, _offset[face].x + p2[0], _offset[face].y + p2[1], _offset[face].z + p2[2], o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];

	// calculate offset quaternion
	this->multiplyQbyQ(q, q2, q1);
}

double Linkbot::getWheelRatio(int standard) {
	switch (standard) {
		case rsLinkbot::BIGWHEEL:
			return _wheel_radius/_bigwheel_radius;
		case rsLinkbot::SMALLWHEEL:
			return _wheel_radius/_smallwheel_radius;
		case rsLinkbot::TINYWHEEL:
			return _wheel_radius/_tinywheel_radius;
	}
	return 0;
}

