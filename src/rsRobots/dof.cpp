#include <rsRobots/Dof>

using namespace rsRobots;
using namespace rsDof;

Dof::Dof(short joint) : Robot(rs::Dof) {
	// enabled joint
	_enabled = joint;

	// body parts
	this->setBodyHeight(0.07250);
	this->setBodyLength(0.03935);
	this->setBodyWidth(0.07835);
	_body_radius = 0.03625;
	_cap_depth = 0.00200;
	_cap_radius = 0.03060;

	// body offsets
	this->addBodyOffset(rs::Pos(0, 0, 0));								// body
	float depth = 0; if (_enabled == Bodies::Face1) depth = _cap_depth/2;
	this->addBodyOffset(rs::Pos(-this->getBodyWidth()/2 - depth, 0, 0));			// face1
	depth = 0; if (_enabled == Bodies::Face2) depth = _cap_depth/2;
	this->addBodyOffset(rs::Pos(0, -this->getBodyLength() - depth, 0));			// face2
	depth = 0; if (_enabled == Bodies::Face3) depth = _cap_depth/2;
	this->addBodyOffset(rs::Pos(this->getBodyWidth()/2 + depth, 0, 0));			// face3

	// connectors
	this->setConnDepth(0.00570);
	this->setConnHeight(0.03715);
	_el_length = 0.13350 + 2*_cap_radius;
	_plank_length = 0.13350;
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Pos Dof::getConnFacePosition(short type, short side, short orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of face
	if (type == Connectors::El) {
		if (side == Connectors::Side2)
			return P.add(Q.multiply(_el_length/4, -this->getBodyWidth()/2 - _cap_depth, 0));
		else if (side == Connectors::Side3)
			return P.add(Q.multiply(_el_length/4, 0, -this->getBodyHeight()/2 - this->getConnDepth()));
		else if (side == Connectors::Side4)
			return P.add(Q.multiply(_el_length/4, 0, this->getBodyHeight()/2 + this->getConnDepth()));
	}
	else if (type == Connectors::Plank)
		return P.add(Q.multiply(0, _plank_length - 2*_cap_radius, 0));

	// default return
	return P;
}

const rs::Quat Dof::getConnFaceQuaternion(short type, short side, short orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get offset of face
	if (type == Connectors::El) {
		if (side == Connectors::Side2)
			return Q.multiply(0, 0, sin(0.785398), cos(0.785398));
		else if (side == Connectors::Side3)
			return Q.multiply(0, sin(0.785398), 0, cos(0.785398));
		else if (side == Connectors::Side4) {
			Q.multiply(0, sin(2.356194), 0, cos(2.356194));
			return Q.multiply(sin(1.570796), 0, 0, cos(1.570796));
		}
	}
	else if (type == Connectors::Plank)
		return Q.multiply(0, 0, sin(1.570796), cos(1.570796));

	// default return
	return Q;
}

const rs::Pos Dof::getConnBodyPosition(short type, short orientation, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// rotate for connector orientation
	rs::Quat Q(this->getConnBodyQuaternion(type, orientation, q));

	// get offset of body
	if (type == Connectors::El)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::Foot)
		return P.add(Q.multiply(this->getConnDepth()/2, 0, 0));
	else if (type == Connectors::Plank)
		return P.add(Q.multiply(this->getConnDepth()/2, _plank_length/2 - _cap_radius, 0));

	// default return
	return P;
}

const rs::Quat Dof::getConnBodyQuaternion(short type, short orientation, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// rotate for orientation
	return Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));
}

const rs::Quat Dof::getRobotBodyQuaternion(short body, float theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// offset quaternion
	if (body == Bodies::Face1)
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
	else if (body == Bodies::Face2)
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));

	// face rotation
	Q.multiply(sin(0.5*theta), 0, 0, cos(0.5*theta));

	// return offset quaternion
	return Q;
}

const rs::Pos Dof::getRobotCenterPosition(short face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// face depth
	float depth = 0;
	if (_enabled == face) depth = _cap_depth;

	// get position of robot
	if (face == Bodies::Face1)
		return P.add(q.multiply(this->getBodyWidth()/2 + depth, 0, 0));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(depth + this->getBodyLength(), 0, 0));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(this->getBodyWidth()/2 + depth, 0, 0));

	// default return
	return P;
}

const rs::Quat Dof::getRobotCenterQuaternion(short face, short orientation, float angle, const rs::Quat &q) {
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

const rs::Pos Dof::getRobotFacePosition(short face, const rs::Pos &p, const rs::Quat &q) {
	// new position
	rs::Pos P(p);

	// face depth
	float depth = 0;
	if (_enabled == face) depth = _cap_depth/2;

	// calculate offset position
	rs::Pos pos = this->getBodyOffset(face);
	if (face == Bodies::Face1)
		return P.add(q.multiply(pos.x() - depth, pos.y(), pos.z()));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(pos.x(), pos.y() - depth, pos.z()));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(pos.x() + depth, pos.y(), pos.z()));

	// default return
	return P;
}

short Dof::getEnabled(void) {
	return _enabled;
}

