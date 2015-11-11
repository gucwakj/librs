#include <rsRobots/Linkbot>

using namespace rsRobots;
using namespace rsLinkbot;

Linkbot::Linkbot(short form) : Robot(form) {
	// disabled joint
	_disabled = -1;
	if (form == rs::LinkbotI) _disabled = Bodies::Joint2;
	else if (form == rs::LinkbotL) _disabled = Bodies::Joint3;

	// body parts
	this->setBodyHeight(0.07250);
	this->setBodyLength(0.03935);
	this->setBodyWidth(0.07835);
	_body_radius = 0.03625;
	_face_depth = 0.00200;
	_face_radius = 0.03060;

	// body offsets
	this->addBodyOffset(rs::Pos(0, 0, 0));		// body
	this->addBodyOffset(rs::Pos(-this->getBodyWidth()/2 - _face_depth/2, 0, 0));	// face1
	this->addBodyOffset(rs::Pos(0, -this->getBodyLength() - _face_depth/2, 0));		// face2
	this->addBodyOffset(rs::Pos(this->getBodyWidth()/2 + _face_depth/2, 0, 0));		// face3

	// connectors
	this->setConnDepth(0.00570);
	this->setConnHeight(0.03715);
	this->setWheelDepth(0.00140);
	this->setWheelRadius(0.04445);
	_bigwheel_radius = 0.05080;
	_bridge_length = 0.13350;
	_cubic_length = 0.07115;
	_omni_length = 0.17360;
	_smallwheel_radius = 0.04445;
	_tinywheel_radius = 0.04128;
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Pos Linkbot::getConnFacePosition(short type, short side, short orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of face
	if (type == Connectors::Bridge)
		return P.add(Q.multiply(0, _bridge_length - 2*_face_radius, 0));
	else if (type == Connectors::Cube) {
		if (side == Connectors::Side2)
			return P.add(Q.multiply(_cubic_length/2, _cubic_length/2, 0));
		else if (side == Connectors::Side3)
			return P.add(Q.multiply(_cubic_length, 0, 0));
		else if (side == Connectors::Side4)
			return P.add(Q.multiply(_cubic_length/2, -_cubic_length/2, 0));
		else if (side == Connectors::Side5)
			return P.add(Q.multiply(_cubic_length/2, 0, _cubic_length/2));
	}
	else if (type == Connectors::DoubleBridge) {
		if (side == Connectors::Side2)
			return P.add(Q.multiply(0, _bridge_length - 2*_face_radius, 0));
		else if (side == Connectors::Side3)
			return P.add(Q.multiply(2*this->getConnDepth(), 0, 0));
		else if (side == Connectors::Side4)
			return P.add(Q.multiply(2*this->getConnDepth(), _bridge_length - 2*_face_radius, 0));
	}
	else if (type == Connectors::Omniplate) {
		if (side == Connectors::Side2)
			return P.add(Q.multiply(0, 0, -_omni_length + 2*_face_radius));
		else if (side == Connectors::Side3)
			return P.add(Q.multiply(0, _omni_length - 2*_face_radius, 0));
		else if (side == Connectors::Side4)
			return P.add(Q.multiply(0, _omni_length - 2*_face_radius, -_omni_length + 2*_face_radius));
	}
	else if (type == Connectors::Simple)
		return P.add(Q.multiply(this->getConnDepth(), 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getConnFaceQuaternion(short type, short side, short orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get offset of face
	if (type == Connectors::Bridge)
		return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (type == Connectors::Cube) {
		if (side == Connectors::Side2)
			return Q.multiply(0, 0, sin(0.785398), cos(0.785398));
		else if (side == Connectors::Side3)
			return Q;
		else if (side == Connectors::Side4)
			return Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
		else if (side == Connectors::Side5) {
			Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
			return Q.multiply(sin(-0.785398), 0, 0, cos(-0.785398));
		}
	}
	else if (type == Connectors::DoubleBridge) {
		if (side == Connectors::Side2)
			return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
		else if (side == Connectors::Side3)
			return Q;
		else if (side == Connectors::Side4)
			return Q;
	}
	else if (type == Connectors::Omniplate)
		return Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (type == Connectors::Simple)
		return Q;

	// default return
	return Q;
}

const rs::Pos Linkbot::getConnBodyPosition(short type, short orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of body
	if (type == Connectors::BigWheel)
		return P.add(Q.multiply(this->getWheelDepth()/2, 0, 0));
	else if (type == Connectors::Bridge)
		return P.add(Q.multiply(this->getConnDepth()/2, _bridge_length/2 - _face_radius, 0));
	else if (type == Connectors::Caster)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::Cube)
		return P.add(Q.multiply(_cubic_length/2, 0, 0));
	else if (type == Connectors::DoubleBridge)
		return P.add(Q.multiply(this->getConnDepth(), _bridge_length/2 - _face_radius, 0));
	else if (type == Connectors::Faceplate)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::Gripper)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::Omniplate)
		return P.add(Q.multiply(this->getConnDepth()/2, _omni_length/2 - _face_radius, -_omni_length/2 + _face_radius));
	else if (type == Connectors::Simple)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::SmallWheel)
		return P.add(Q.multiply(this->getWheelDepth()/2, 0, 0));
	else if (type == Connectors::TinyWheel)
		return P.add(Q.multiply(this->getWheelDepth()/2, 0, 0));
	else if (type == Connectors::Wheel)
		return P.add(Q.multiply(this->getWheelDepth()/2, 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getConnBodyQuaternion(short type, short orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	return Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));
}

const rs::Quat Linkbot::getRobotBodyQuaternion(short body, float theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// offset quaternion
	if (body == Bodies::Cap1)
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (body == Bodies::Cap2)
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));

	// face rotation
	Q.multiply(sin(0.5*theta), 0, 0, cos(0.5*theta));

	// return offset quaternion
	return Q;
}

const rs::Pos Linkbot::getRobotCenterPosition(short face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// get position of robot
	if (face == Bodies::Face1)
		return P.add(q.multiply(this->getBodyWidth()/2 + _face_depth, 0, 0));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(_face_depth + this->getBodyLength(), 0, 0));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(this->getBodyWidth()/2 + _face_depth, 0, 0));

	// default return
	return P;
}

const rs::Quat Linkbot::getRobotCenterQuaternion(short face, short orientation, float angle, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get quaternion of robot
	if (face == Bodies::Face1)
		Q.multiply(sin(0.5*angle), 0, 0, cos(0.5*angle));
	else if (face == Bodies::Face2) {
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
		Q.multiply(sin(0.5*angle), 0, 0, cos(0.5*angle));
	}
	else if (face == Bodies::Face3) {
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
		Q.multiply(sin(-0.5*angle), 0, 0, cos(-0.5*angle));
	}

	// return offset quaternion
	return Q;
}

const rs::Pos Linkbot::getRobotFacePosition(short face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// calculate offset position
	rs::Pos pos = this->getBodyOffset(face);
	if (face == Bodies::Face1)
		return P.add(q.multiply(pos.x() - _face_depth/2, pos.y(), pos.z()));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(pos.x(), pos.y() - _face_depth/2, pos.z()));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(pos.x() + _face_depth/2, pos.y(), pos.z()));

	// default return
	return P;
}

float Linkbot::getWheelRatio(short standard) {
	float radius = this->getWheelRadius();
	switch (standard) {
		case Connectors::BigWheel:
			return radius/_bigwheel_radius;
		case Connectors::SmallWheel:
			return radius/_smallwheel_radius;
		case Connectors::TinyWheel:
			return radius/_tinywheel_radius;
	}
	return 0;
}

float Linkbot::getCasterScale(void) {
	return this->getWheelRadius()/(this->getBodyHeight() + 0.008);
}

const rs::Quat Linkbot::tiltForWheels(short type1, short type2, float &p2) {
	// set wheel on this face
	this->setWheelLeft(type1);
	this->setWheelRight(type2);

	// tilt
	if (type1 == Connectors::None && type2 == Connectors::TinyWheel) {
		p2 = 0.002117;
		return rs::Quat(-0.042648, -0.028636, 0.001300, 0.998679);
	}
	else if (type1 == Connectors::None && type2 == Connectors::SmallWheel) {
		p2 = 0.003400;
		return rs::Quat(-0.034840, -0.046668, 0.001485, 0.9);
	}
	else if (type1 == Connectors::None && type2 == Connectors::BigWheel) {
		p2 = 0.005581;
		return rs::Quat(0, -0.082108, 0, 0.996400);
	}
	else if (type1 == Connectors::TinyWheel && type2 == Connectors::None) {
		p2 = 0.002117;
		return rs::Quat(-0.042648, 0.028636, -0.001300, 0.998679);
	}
	else if (type1 == Connectors::TinyWheel && type2 == Connectors::TinyWheel) {
		p2 = 0.004909;
		return rs::Quat(-0.0277827, 0, 0, 0.999614);
	}
	else if (type1 == Connectors::TinyWheel && type2 == Connectors::SmallWheel) {
		p2 = 0.006137;
		return rs::Quat(-0.021193, -0.017343, 0.000250, 0.999625);
	}
	else if (type1 == Connectors::TinyWheel && type2 == Connectors::BigWheel) {
		p2 = 0.009423;
		return rs::Quat(-0.002151, -0.047644, 0, 0.998862);
	}
	else if (type1 == Connectors::SmallWheel && type2 == Connectors::None) {
		p2 = 0.003400;
		return rs::Quat(-0.034840, 0.046668, -0.001485, 0.9);
	}
	else if (type1 == Connectors::SmallWheel && type2 == Connectors::TinyWheel) {
		p2 = 0.006137;
		return rs::Quat(-0.021193, 0.017343, -0.00025, 0.999625);
	}
	else if (type1 == Connectors::SmallWheel && type2 == Connectors::SmallWheel) {
		p2 = 0.007781;
		return rs::Quat(-0.012152, 0, 0, 0.999926);
	}
	else if (type1 == Connectors::SmallWheel && type2 == Connectors::BigWheel) {
		p2 = 0.011222;
		return rs::Quat(0.007552, -0.031155, -0.000203, 0.999486);
	}
	else if (type1 == Connectors::BigWheel && type2 == Connectors::None) {
		p2 = 0.005581;
		return rs::Quat(0, 0.082108, 0, 0.996400);
	}
	else if (type1 == Connectors::BigWheel && type2 == Connectors::TinyWheel) {
		p2 = 0.009423;
		return rs::Quat(-0.002151, 0.04764, 0, 0.998862);
	}
	else if (type1 == Connectors::BigWheel && type2 == Connectors::SmallWheel) {
		p2 = 0.011222;
		return rs::Quat(0.007552, 0.031155, 0.000203, 0.9);
	}
	else if (type1 == Connectors::BigWheel && type2 == Connectors::BigWheel) {
		p2 = 0.014469;
		return rs::Quat(0.025255, 0, 0, 0.999681);
	}

	// return default
	p2 = 0;
	return rs::Quat();
}

