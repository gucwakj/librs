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
	_radius = _body_height/2;
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
void LinkbotT::getOffsetQuat(int body, const double *q1, double *q3) {
	// offset quaternion
	double q2[4] = {q1[0], q1[1], q1[2], q1[3]};

	// special quaternion for faces
	switch (body) {
		case FACE1:
		case FACE3:
			q2[0] = 0;
			q2[1] = sin(0.785398);	// PI/4
			q2[2] = 0;
			q2[3] = cos(0.785398);	// PI/4
			break;
		case FACE2:
			q2[0] = sin(0.785398);	// PI/4
			q2[1] = 0;
			q2[2] = 0;
			q2[3] = cos(0.785398);	// PI/4
			break;
	}

	// calculate body part quaternion
	q3[0] = q2[3]*q1[0] + q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1];
	q3[1] = q2[3]*q1[1] - q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0];
	q3[2] = q2[3]*q1[2] + q2[0]*q1[1] - q2[1]*q1[0] + q2[2]*q1[3];
	q3[3] = q2[3]*q1[3] - q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2];
}

