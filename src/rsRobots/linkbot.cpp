#include <rsRobots/Linkbot>

using namespace rsRobots;
using namespace rsLinkbot;

Linkbot::Linkbot(int form) : Robot(form) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_disabled = -1;
	if (form == rs::LINKBOTI) _disabled = JOINT2;
	else if (form == rs::LINKBOTL) _disabled = JOINT3;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_conn_depth = 0.00570;
	_conn_height = 0.03715;
	_bigwheel_radius = 0.05080;
	_bridge_length = 0.13350;
	_cubic_length = 0.07115;
	_omni_length = 0.17360;
	_salamander_length = _bridge_length + 2*_face_radius;
	_smallwheel_radius = 0.04445;
	_tinywheel_radius = 0.04128;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;
	_offset.push_back(rs::Pos(0, 0, 0));									// body
	_offset.push_back(rs::Pos(-_body_width/2 - _face_depth/2, 0, 0));		// face1
	_offset.push_back(rs::Pos(0, -_body_length - _face_depth/2, 0));		// face2
	_offset.push_back(rs::Pos(_body_width/2 + _face_depth/2, 0, 0));		// face3
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Pos Linkbot::getConnFacePosition(int type, int side, int orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of face
	if (type == rsLinkbot::BRIDGE)
		return P.add(Q.multiply(0, _bridge_length - 2*_face_radius, 0));
	else if (type == rsLinkbot::CUBE) {
		if (side == SIDE2)
			return P.add(Q.multiply(_cubic_length/2, _cubic_length/2, 0));
		else if (side == SIDE3)
			return P.add(Q.multiply(_cubic_length, 0, 0));
		else if (side == SIDE4)
			return P.add(Q.multiply(_cubic_length/2, -_cubic_length/2, 0));
		else if (side == SIDE5)
			return P.add(Q.multiply(_cubic_length/2, 0, _cubic_length/2));
	}
	else if (type == rsLinkbot::DOUBLEBRIDGE) {
		if (side == SIDE2)
			return P.add(Q.multiply(0, _bridge_length - 2*_face_radius, 0));
		else if (side == SIDE3)
			return P.add(Q.multiply(2*_conn_depth, 0, 0));
		else if (side == SIDE4)
			return P.add(Q.multiply(2*_conn_depth, _bridge_length - 2*_face_radius, 0));
	}
	else if (type == rsLinkbot::EL) {
		if (side == SIDE2)
			return P.add(Q.multiply(_salamander_length/4, -_body_width/2 - _face_depth, 0));
		else if (side == SIDE3)
			return P.add(Q.multiply(_salamander_length/4, 0, -_body_height/2 - _conn_depth));
		else if (side == SIDE4)
			return P.add(Q.multiply(_salamander_length/4, 0, _body_height/2 + _conn_depth));
	}
	else if (type == rsLinkbot::OMNIPLATE) {
		if (side == SIDE2)
			return P.add(Q.multiply(0, 0, -_omni_length + 2*_face_radius));
		else if (side == SIDE3)
			return P.add(Q.multiply(0, _omni_length - 2*_face_radius, 0));
		else if (side == SIDE4)
			return P.add(Q.multiply(0, _omni_length - 2*_face_radius, -_omni_length + 2*_face_radius));
	}
	else if (type == rsLinkbot::SALAMANDER)
		return P.add(Q.multiply(-_face_depth - _body_width/2, _salamander_length, 0));
	else if (type == rsLinkbot::SIMPLE)
		return P.add(Q.multiply(_conn_depth, 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getConnFaceQuaternion(int type, int side, int orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get offset of face
	if (type == rsLinkbot::BRIDGE)
		return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (type == rsLinkbot::CUBE) {
		if (side == SIDE2)
			return Q.multiply(0, 0, sin(0.785398), cos(0.785398));
		else if (side == SIDE3)
			return Q;
		else if (side == SIDE4)
			return Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
		else if (side == SIDE5) {
			Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
			return Q.multiply(sin(-0.785398), 0, 0, cos(-0.785398));
		}
	}
	else if (type == rsLinkbot::DOUBLEBRIDGE) {
		if (side == SIDE2)
			return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
		else if (side == SIDE3)
			return Q;
		else if (side == SIDE4)
			return Q;
	}
	else if (type == rsLinkbot::EL) {
		if (side == SIDE2)
			return Q.multiply(0, 0, sin(0.785398), cos(0.785398));
		else if (side == SIDE3)
			return Q.multiply(0, sin(0.785398), 0, cos(0.785398));
		else if (side == SIDE4) {
			Q.multiply(0, sin(2.356194), 0, cos(2.356194));
			return Q.multiply(sin(1.570796), 0, 0, cos(1.570796));
		}
	}
	else if (type == rsLinkbot::OMNIPLATE)
		return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (type == rsLinkbot::SALAMANDER)
		return Q.multiply(0, 0, sin(2.356194), cos(2.356194));
	else if (type == rsLinkbot::SIMPLE)
		return Q;

	// default return
	return Q;
}

const rs::Pos Linkbot::getConnBodyPosition(int type, int orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of body
	if (type == rsLinkbot::BIGWHEEL)
		return P.add(Q.multiply(_wheel_depth/2, 0, 0));
	else if (type == rsLinkbot::BRIDGE)
		return P.add(Q.multiply(_conn_depth/2, _bridge_length/2 - _face_radius, 0));
	else if (type == rsLinkbot::CASTER)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == rsLinkbot::CUBE)
		return P.add(Q.multiply(_cubic_length/2, 0, 0));
	else if (type == rsLinkbot::DOUBLEBRIDGE)
		return P.add(Q.multiply(_conn_depth, _bridge_length/2 - _face_radius, 0));
	else if (type == rsLinkbot::EL)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == rsLinkbot::FACEPLATE)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == rsLinkbot::GRIPPER)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == rsLinkbot::OMNIPLATE)
		return P.add(Q.multiply(_conn_depth/2, _omni_length/2 - _face_radius, -_omni_length/2 + _face_radius));
	else if (type == rsLinkbot::SALAMANDER)
		return P.add(Q.multiply(-_face_depth - _body_width/2, _salamander_length/2, 0));
	else if (type == rsLinkbot::SIMPLE)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == rsLinkbot::SMALLWHEEL)
		return P.add(Q.multiply(_wheel_depth/2, 0, 0));
	else if (type == rsLinkbot::TINYWHEEL)
		return P.add(Q.multiply(_wheel_depth/2, 0, 0));
	else if (type == rsLinkbot::WHEEL)
		return P.add(Q.multiply(_wheel_depth/2, 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getConnBodyQuaternion(int type, int orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	return Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));
}

const rs::Quat Linkbot::getRobotBodyQuaternion(int body, double theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// offset quaternion
	if (body == FACE1)
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (body == FACE2)
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));

	// face rotation
	Q.multiply(sin(0.5*theta), 0, 0, cos(0.5*theta));

	// return offset quaternion
	return Q;
}

const rs::Pos Linkbot::getRobotCenterPosition(int face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// get position of robot
	if (face == FACE1)
		return P.add(q.multiply(_body_width/2 + _face_depth, 0, 0));
	else if (face == FACE2)
		return P.add(q.multiply(_face_depth + _body_length, 0, 0));
	else if (face == FACE3)
		return P.add(q.multiply(_body_width/2 + _face_depth, 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getRobotCenterQuaternion(int face, int orientation, double angle, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get quaternion of robot
	if (face == FACE1)
		Q.multiply(sin(0.5*angle), 0, 0, cos(0.5*angle));
	else if (face == FACE2) {
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
		Q.multiply(sin(0.5*angle), 0, 0, cos(0.5*angle));
	}
	else if (face == FACE3) {
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
		Q.multiply(sin(-0.5*angle), 0, 0, cos(-0.5*angle));
	}

	// return offset quaternion
	return Q;
}

const rs::Pos Linkbot::getRobotFacePosition(int face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// calculate offset position
	if (face == FACE1)
		return P.add(q.multiply(_offset[face].x() - _face_depth/2, _offset[face].y(), _offset[face].z()));
	else if (face == FACE2)
		return P.add(q.multiply(_offset[face].x(), _offset[face].y() - _face_depth/2, _offset[face].z()));
	else if (face == FACE3)
		return P.add(q.multiply(_offset[face].x() + _face_depth/2, _offset[face].y(), _offset[face].z()));

	// default return
	return P;
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

double Linkbot::getCasterScale(void) {
	return _wheel_radius/(_body_height + 0.008);
}

double Linkbot::riseByWheels(int wheel_type, double radius) {
	if (wheel_type == TINYWHEEL)
		return _tinywheel_radius - _body_height/2;
	else if (wheel_type == SMALLWHEEL)
		return _smallwheel_radius - _body_height/2;
	else if (wheel_type == BIGWHEEL)
		return _bigwheel_radius - _body_height/2;
	else if (wheel_type == WHEEL)
		return radius - _body_height/2;
	return -1;
}

const rs::Quat Linkbot::tiltForWheels(int type, int face, double &p2, double radius) {
	// invalid type
	if (type == 0) {
		p2 = 0;
		return rs::Quat();
	}

	// get other wheel joint
	int other = 0;
	if (face == FACE1) other = 1;

	// tilt left or right based upon face
	int i = 1;
	if (face == FACE3) i = -1;

	// set wheel on this face
	_wheels[(other ? 0 : 1)] = type;

	// tilt
	if (type == TINYWHEEL) {
		if (_wheels[other] == TINYWHEEL) {
			p2 = 0.0029595;
			double q[4] = {0, 0.0285493, 0, 0.999592};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat().multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == SMALLWHEEL) {
			p2 = 0.0030787;
			double q[4] = {0, 0.0469256, 0, 0.998898};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, -i*0.0171147, 0, 0.999851).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == BIGWHEEL) {
			p2 = 0.0037111;
			double q[4] = {0, 0.0820308, 0, 0.996629};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, -i*0.0480794, 0, 0.998843).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else {
			p2 = 0.0020705;
			return rs::Quat(0, i*0.0285493, 0, 0.999592);
		}
	}
	else if (type == SMALLWHEEL) {
		if (_wheels[other] == TINYWHEEL) {
			p2 = 0.0043264;
			double q[4] = {0, 0.0285493, 0, 0.999592};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, i*0.0171147, 0, 0.999851).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == SMALLWHEEL) {
			p2 = 0.0048818;
			double q[4] = {0, 0.0469256, 0, 0.998898};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat().multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == BIGWHEEL) {
			p2 = 0.0078349;
			double q[4] = {0, 0.0820308, 0, 0.996629};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, -i*0.0326405, 0, 0.999466).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else {
			p2 = 0.003318;
			return rs::Quat(0, i*0.0469256, 0, 0.998898);
		}
	}
	else if (type == BIGWHEEL) {
		if (_wheels[other] == TINYWHEEL) {
			p2 = 0.0074497;
			double q[4] = {0, -i*0.0285493, 0, 0.999592};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, i*0.0480794, 0, 0.998843).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == SMALLWHEEL) {
			p2 = 0.0078349;
			double q[4] = {0, -i*0.0469256, 0, 0.998898};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat(0, i*0.0326405, 0, 0.999466).multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else if (_wheels[other] == BIGWHEEL) {
			p2 = 0.0087409;
			double q[4] = {0, -i*0.0820308, 0, 0.996629};
			double length = q[1]*q[1] + q[3]*q[3];
			return rs::Quat().multiply(rs::Quat(0, i*q[1]/length, 0, q[3]/length));
		}
		else {
			p2 = 0.0058091;
			return rs::Quat(0, i*0.0820308, 0, 0.996629);
		}
	}

	// return default
	p2 = 0;
	return rs::Quat();
}

