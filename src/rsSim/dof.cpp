#include <iostream>
#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Dof>

using namespace rsSim;
using namespace rsDof;

Dof::Dof(int joint) : rsRobots::Robot(rs::DOF), rsRobots::Dof(joint) {
	// degrees of freedom
	_dof = Bodies::Num_Joints;

	// create arrays
	_motor.resize(_dof);

	// fill with default data
	_motor[Bodies::Joint].accel.init = 0;
	_motor[Bodies::Joint].accel.run = 0;
	_motor[Bodies::Joint].accel.period = 0;
	_motor[Bodies::Joint].accel.start = 0;
	_motor[Bodies::Joint].alpha = 0;
	_motor[Bodies::Joint].encoder = rs::D2R(0.25);
	_motor[Bodies::Joint].goal = 0;
	_motor[Bodies::Joint].mode = SEEK;
	_motor[Bodies::Joint].offset = 0;
	_motor[Bodies::Joint].omega = 0.7854;			// 45 deg/sec
	_motor[Bodies::Joint].omega_max = 4.1888;		// 240 deg/sec
	_motor[Bodies::Joint].record = false;
	_motor[Bodies::Joint].record_active = false;
	_motor[Bodies::Joint].record_num = 0;
	_motor[Bodies::Joint].safety_angle = 10;
	_motor[Bodies::Joint].safety_timeout = 4;
	_motor[Bodies::Joint].starting = 0;
	_motor[Bodies::Joint].state = NEUTRAL;
	_motor[Bodies::Joint].stopping = 0;
	_motor[Bodies::Joint].success = true;
	_motor[Bodies::Joint].tau_max = 2;
	_motor[Bodies::Joint].timeout = 0;
	_motor[Bodies::Joint].theta = 0;
	MUTEX_INIT(&_motor[Bodies::Joint].success_mutex);
	COND_INIT(&_motor[Bodies::Joint].success_cond);
	_connected = 0;
	_distOffset = 0;
	_id = -1;
	_motion = false;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 1;
	_sim = NULL;
	_speed = 2;
	_trace = 1;
}

Dof::~Dof(void) {
	// delete mutexes
	MUTEX_DESTROY(&_motor[Bodies::Joint].success_mutex);
	COND_DESTROY(&_motor[Bodies::Joint].success_cond);
}

void Dof::moveJointOnce(double *values) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set motion parameters
	_motor[Bodies::Joint].mode = SINGULAR;
	_motor[Bodies::Joint].state = POSITIVE;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[Bodies::Joint].id);
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, _motor[Bodies::Joint].theta);
	dBodyEnable(_body[0]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set array
	_index = 0;
	_values = values;

	// unsuccessful
	MUTEX_LOCK(&_motor[Bodies::Joint].success_mutex);
	_motor[Bodies::Joint].success = false;
	MUTEX_UNLOCK(&_motor[Bodies::Joint].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	inherited functions
 **********************************************************/
int Dof::addConnector(int type, int face, int orientation, double size, int side, int conn) {
	// get connector body position
	rs::Pos P1 = this->getRobotFacePosition(face, this->getPosition(), this->getQuaternion());
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, (_enabled == face) ? _motor[Bodies::Joint].theta : 0, this->getQuaternion());
	if (conn == -1) {
		P1 = this->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = this->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = this->getConnFaceQuaternion(type, side, orientation, Q1);
		P1 = this->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
	}

	// create new connector
	_conn.push_back(Connector());

	// daisy chained or not
	if (conn == -1) {
		_conn.back().d_side = -1;
		_conn.back().d_type = -1;
		_conn.back().face = face;
		_conn.back().type = type;
		_conn.back().body = dBodyCreate(_world);
		_conn.back().orientation = orientation;
	}
	else {
		_conn.back().d_side = side;
		_conn.back().d_type = type;
		_conn.back().face = face;
		_conn.back().type = conn;
		_conn.back().body = dBodyCreate(_world);
		_conn.back().orientation = orientation;
		type = conn;
	}

	// build connector
	switch (type) {
		case Connectors::El:
			this->build_el(_conn.back());
			break;
		case Connectors::Foot:
			this->build_foot(_conn.back());
			break;
	}

	// set body parameters
	dBodySetPosition(_conn.back().body, P1[0], P1[1], P1[2]);
	dQuaternion Q = {Q1[3], Q1[0], Q1[1], Q1[2]};
	dBodySetQuaternion(_conn.back().body, Q);

	// fix connector to body
	this->fix_connector_to_body(Bodies::Cap, _conn.back().body, conn);

	// success
	return 0;
}

int Dof::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// build
	this->buildIndividual(p, q, a);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int Dof::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, dBodyID base, int face, int orientation, int ground) {
	// calculate center of robot location
	rs::Pos P = this->getRobotCenterPosition(face, p, q);
	rs::Quat Q = this->getRobotCenterQuaternion(face, orientation, rs::D2R(a[0]), q);

	// build new module
	this->buildIndividual(P, Q, a);

	// add fixed joint to attach two modules
	this->fix_body_to_connector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int Dof::buildIndividual(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a) {
	// init body parts
	for (int i = 0; i < Bodies::Num_Parts; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angle to radians
	_motor[Bodies::Joint].goal = _motor[Bodies::Joint].theta = rs::D2R(a[0]);

	// build robot bodies
	this->build_body(p, q);
	this->build_cap(this->getRobotBodyPosition(_enabled, p, q), this->getRobotBodyQuaternion(_enabled, 0, q));

	// build joints
	rs::Pos o;
	_motor[Bodies::Joint].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[Bodies::Joint].joint, _body[Bodies::Body], _body[Bodies::Cap]);
	if (_enabled == Bodies::Face1) {
		o = q.multiply(-_body_width/2, 0, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(1, 0, 0);
	}
	else if (_enabled == Bodies::Face2) {
		o = q.multiply(0, -_body_length, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
	}
	else if (_enabled == Bodies::Face3) {
		o = q.multiply(_body_width/2, 0, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
	}
	dJointSetHingeAxis(_motor[Bodies::Joint].joint, o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[Bodies::Cap], o[0], o[1], o[2]);

	// build rotated joints
	if (_motor[Bodies::Joint].theta != 0) this->build_cap(this->getRobotBodyPosition(_enabled, p, q), this->getRobotBodyQuaternion(_enabled, _motor[Bodies::Joint].theta, q));

	// set motors
	_motor[Bodies::Joint].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[Bodies::Joint].id, _body[Bodies::Body], _body[Bodies::Cap]);
	dJointSetAMotorMode(_motor[Bodies::Joint].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[Bodies::Joint].id, 1);
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, 0);
	dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamFMax, _motor[Bodies::Joint].tau_max);
	dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamFudgeFactor, 0.3);
	if (_enabled == Bodies::Face1)
		o = q.multiply(1, 0, 0);
	else if (_enabled == Bodies::Face2)
		o = q.multiply(0, 1, 0);
	else if (_enabled == Bodies::Face3)
		o = q.multiply(-1, 0, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < Bodies::Num_Parts; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

double Dof::calculate_angle(int id) {
	return mod_angle(_motor[Bodies::Joint].theta, dJointGetHingeAngle(_motor[Bodies::Joint].joint), dJointGetHingeAngleRate(_motor[Bodies::Joint].joint)) - _motor[Bodies::Joint].offset;
}

dBodyID Dof::getBodyID(int face) {
	if (face == _enabled) return _body[Bodies::Cap];
	return _body[Bodies::Body];
}

const rs::Pos Dof::getCoM(double &mass) {
	double total = 0;
	double x = 0, y = 0, z = 0;
	// body parts
	for (unsigned int i = 0; i < _body.size(); i++) {
		dMass m;
		dBodyGetMass(_body[i], &m);
		const double *p = dBodyGetPosition(_body[i]);
		x += m.mass*p[0];
		y += m.mass*p[1];
		z += m.mass*p[2];
		total += m.mass;
	}
	// connectors
	for (unsigned int i = 0; i < _conn.size(); i++) {
		dMass m;
		dBodyGetMass(_conn[i].body, &m);
		const double *p = dBodyGetPosition(_conn[i].body);
		x += m.mass*p[0];
		y += m.mass*p[1];
		z += m.mass*p[2];
		total += m.mass;
	}
	x /= total;
	y /= total;
	z /= total;

	mass = total;
	return rs::Pos(x, y, z);
}

const rs::Vec Dof::getJoints(void) {
	return rs::Vec(_motor[Bodies::Joint].theta);
}

void Dof::init_params(void) { }

void Dof::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// store current angle
	_motor[Bodies::Joint].theta = calculate_angle(_enabled);
	// set motor angle to current angle
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, _motor[Bodies::Joint].theta);
	// engage motor depending upon motor mode
	double step = _sim->getStep();
	switch (_motor[Bodies::Joint].mode) {
		case SEEK:
			if ((_motor[Bodies::Joint].goal - 6*_motor[Bodies::Joint].encoder - _motor[Bodies::Joint].theta) > rs::Epsilon) {
				_motor[Bodies::Joint].state = POSITIVE;
				if (_motor[Bodies::Joint].starting++ < 25)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, _motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/50);
				else if (_motor[Bodies::Joint].starting++ < 50)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, _motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/150 + 0.3*fabs(_motor[Bodies::Joint].omega));
				else if (_motor[Bodies::Joint].starting++ < 100)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, _motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/150 + fabs(_motor[Bodies::Joint].omega)/3);
				else
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, fabs(_motor[Bodies::Joint].omega));
			}
			else if ((_motor[Bodies::Joint].goal - 3*_motor[Bodies::Joint].encoder - _motor[Bodies::Joint].theta) > rs::Epsilon) {
				_motor[Bodies::Joint].state = POSITIVE;
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, fabs(_motor[Bodies::Joint].omega)/2);
			}
			else if ((_motor[Bodies::Joint].goal - _motor[Bodies::Joint].encoder/2 - _motor[Bodies::Joint].theta) > rs::Epsilon) {
				_motor[Bodies::Joint].state = POSITIVE;
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, fabs(_motor[Bodies::Joint].omega)/4);
			}
			else if ((_motor[Bodies::Joint].theta - _motor[Bodies::Joint].goal - 6*_motor[Bodies::Joint].encoder) > rs::Epsilon) {
				_motor[Bodies::Joint].state = NEGATIVE;
				if (_motor[Bodies::Joint].starting++ < 25)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -_motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/50);
				else if (_motor[Bodies::Joint].starting++ < 50)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -_motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/150 - 0.3*fabs(_motor[Bodies::Joint].omega));
				else if (_motor[Bodies::Joint].starting++ < 100)
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -_motor[Bodies::Joint].starting*fabs(_motor[Bodies::Joint].omega)/150 - fabs(_motor[Bodies::Joint].omega)/3);
				else
					dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -fabs(_motor[Bodies::Joint].omega));
			}
			else if ((_motor[Bodies::Joint].theta - _motor[Bodies::Joint].goal - 3*_motor[Bodies::Joint].encoder) > rs::Epsilon) {
				_motor[Bodies::Joint].state = NEGATIVE;
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -fabs(_motor[Bodies::Joint].omega)/2);
			}
			else if ((_motor[Bodies::Joint].theta - _motor[Bodies::Joint].goal - _motor[Bodies::Joint].encoder/2) > rs::Epsilon) {
				_motor[Bodies::Joint].state = NEGATIVE;
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, -fabs(_motor[Bodies::Joint].omega)/4);
			}
			else {
				_motor[Bodies::Joint].state = HOLD;
				_motor[Bodies::Joint].starting = 0;
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, 0);
			}
			break;
		case SINGULAR:
			// reenable body on start
			dBodyEnable(_body[0]);

			// set new omega
			_motor[Bodies::Joint].goal = _values[_index++];
			_motor[Bodies::Joint].omega = (_motor[Bodies::Joint].goal - _motor[Bodies::Joint].theta)/step;
			_motor[Bodies::Joint].state = NEUTRAL;

			// move forever
			dJointEnable(_motor[Bodies::Joint].id);
			dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, _motor[Bodies::Joint].omega);

			// end
			break;
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void Dof::simPostCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);
	MUTEX_LOCK(&_success_mutex);

	// lock mutex
	MUTEX_LOCK(&_motor[Bodies::Joint].success_mutex);
	// zero velocity == stopped
	_motor[Bodies::Joint].stopping += (!(int)(dJointGetAMotorParam(_motor[Bodies::Joint].id, dParamVel)*1000) );
	// once motor has been stopped for 10 steps
	if (_motor[Bodies::Joint].stopping == 50) {
		_motor[Bodies::Joint].stopping = 0;
		_motor[Bodies::Joint].success = 1;
	}
	// signal success
	if (_motor[Bodies::Joint].success)
		COND_SIGNAL(&_motor[Bodies::Joint].success_cond);
	// unlock mutex
	MUTEX_UNLOCK(&_motor[Bodies::Joint].success_mutex);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void Dof::build_body(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m, m1;
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, 0, -_body_length/2, 0);
	dMassSetCylinder(&m1, 400, 1, _body_radius, _body_width);
	dMassAdd(&m, &m1);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[Bodies::Body], &m);

	// set body parameters
	dBodySetPosition(_body[Bodies::Body], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[Bodies::Body], Q);
	dBodySetFiniteRotationMode(_body[Bodies::Body], 1);

	// build geometries
	dGeomID geom[2];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, _body_width, _body_length, _body_height);
	dGeomSetBody(geom[0], _body[Bodies::Body]);
	dGeomSetOffsetPosition(geom[0], 0, -_body_length/2, 0);

	// set geometry 1 - cylinder
	geom[1] = dCreateCylinder(_space, _body_radius, _body_width);
	dGeomSetBody(geom[1], _body[Bodies::Body]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom[1], Q1);
}

void Dof::build_cap(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m;
	if (_enabled == Bodies::Face2)
		dMassSetCylinder(&m, 270, 2, 2*_cap_radius, _cap_depth);
	else
		dMassSetCylinder(&m, 270, 1, 2*_cap_radius, _cap_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[Bodies::Cap], &m);

	// set body parameters
	dBodySetPosition(_body[Bodies::Cap], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[Bodies::Cap], Q);
	dBodySetFiniteRotationMode(_body[Bodies::Cap], 1);

	// destroy old geom when rotating joint on rebuild
	dGeomID geom1 = dBodyGetFirstGeom(_body[Bodies::Cap]);
	while (geom1) {
		dGeomDestroy(geom1);
		geom1 = dBodyGetNextGeom(geom1);
	}

	// set geometry
	dGeomID geom = dCreateCylinder(_space, _cap_radius, _cap_depth);
	dGeomSetBody(geom, _body[Bodies::Cap]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

void Dof::build_el(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_cap_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, 2*_cap_radius, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Dof::build_foot(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_cap_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, 2*_cap_radius, _conn_height);
	dGeomSetBody(geom, conn.body);
}

