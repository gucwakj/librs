#include <rsRobots/Dof>

using namespace rsRobots;
using namespace rsDof;

Dof::Dof(short joint) : Robot(rs::Dof) {
	// enabled joint
	_enabled = joint;

	// body parts
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_cap_depth = 0.00200;
	_cap_radius = 0.03060;

	// body position offsets
	_offset.push_back(rs::Pos(0, 0, 0));								// body
	float depth = 0; if (_enabled == Bodies::Face1) depth = _cap_depth/2;
	_offset.push_back(rs::Pos(-_body_width/2 - depth, 0, 0));			// face1
	depth = 0; if (_enabled == Bodies::Face2) depth = _cap_depth/2;
	_offset.push_back(rs::Pos(0, -_body_length - depth, 0));			// face2
	depth = 0; if (_enabled == Bodies::Face3) depth = _cap_depth/2;
	_offset.push_back(rs::Pos(_body_width/2 + depth, 0, 0));			// face3

	// connectors
	_conn_depth = 0.00570;
	_conn_height = 0.03715;
	_el_length = 0.13350 + 2*_cap_radius;
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
			return P.add(Q.multiply(_el_length/4, -_body_width/2 - _cap_depth, 0));
		else if (side == Connectors::Side3)
			return P.add(Q.multiply(_el_length/4, 0, -_body_height/2 - _conn_depth));
		else if (side == Connectors::Side4)
			return P.add(Q.multiply(_el_length/4, 0, _body_height/2 + _conn_depth));
	}

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
		return P.add(Q.multiply(_conn_depth/2, 0, 0));
	else if (type == Connectors::Foot)
		return P.add(Q.multiply(_conn_depth/2, 0, 0));

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
		return P.add(q.multiply(_body_width/2 + depth, 0, 0));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(depth + _body_length, 0, 0));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(_body_width/2 + depth, 0, 0));

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
	if (face == Bodies::Face1)
		return P.add(q.multiply(_offset[face].x() - depth, _offset[face].y(), _offset[face].z()));
	else if (face == Bodies::Face2)
		return P.add(q.multiply(_offset[face].x(), _offset[face].y() - depth, _offset[face].z()));
	else if (face == Bodies::Face3)
		return P.add(q.multiply(_offset[face].x() + depth, _offset[face].y(), _offset[face].z()));

	// default return
	return P;
}

short Dof::getEnabled(void) {
	return _enabled;
}

