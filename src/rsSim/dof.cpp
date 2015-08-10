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
	_motor[0].accel.init = 0;
	_motor[0].accel.run = 0;
	_motor[0].accel.period = 0;
	_motor[0].accel.start = 0;
	_motor[0].alpha = 0;
	_motor[0].encoder = rs::D2R(0.25);
	_motor[0].goal = 0;
	_motor[0].mode = SEEK;
	_motor[0].offset = 0;
	_motor[0].omega = 0.7854;			// 45 deg/sec
	_motor[0].omega_max = 4.1888;		// 240 deg/sec
	_motor[0].record = false;
	_motor[0].record_active = false;
	_motor[0].record_num = 0;
	_motor[0].safety_angle = 10;
	_motor[0].safety_timeout = 4;
	_motor[0].starting = 0;
	_motor[0].state = NEUTRAL;
	_motor[0].stopping = 0;
	_motor[0].success = true;
	_motor[0].tau_max = 2;
	_motor[0].timeout = 0;
	_motor[0].theta = 0;
	MUTEX_INIT(&_motor[0].success_mutex);
	COND_INIT(&_motor[0].success_cond);
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
	MUTEX_DESTROY(&_motor[0].success_mutex);
	COND_DESTROY(&_motor[0].success_cond);
}

void Dof::moveJointOnce(double *values) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set motion parameters
	_motor[0].mode = SINGULAR;
	_motor[0].state = POSITIVE;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[0].id);
	dJointSetAMotorAngle(_motor[0].id, 0, _motor[0].theta);
	dBodyEnable(_body[0]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set array
	_loc = 0;
	_values = values;

	// unsuccessful
	MUTEX_LOCK(&_motor[0].success_mutex);
	_motor[0].success = false;
	MUTEX_UNLOCK(&_motor[0].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	inherited functions
 **********************************************************/
int Dof::addConnector(int type, int face, int orientation, double size, int side, int conn) {
	// get connector body position
	rs::Pos P1 = this->getRobotFacePosition(face, this->getPosition(), this->getQuaternion());
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, (_enabled == face) ? _motor[0].theta : 0, this->getQuaternion());
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
	_motor[0].goal = _motor[0].theta = rs::D2R(a[0]);

	// build robot bodies
	this->build_body(p, q);
	this->build_cap(this->getRobotBodyPosition(_enabled, p, q), this->getRobotBodyQuaternion(_enabled, 0, q));

	// build joints
	rs::Pos o;
	_motor[0].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[0].joint, _body[Bodies::Body], _body[Bodies::Cap]);
	if (_enabled == Bodies::Face1) {
		o = q.multiply(-_body_width/2, 0, 0);
		dJointSetHingeAnchor(_motor[0].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(1, 0, 0);
	}
	else if (_enabled == Bodies::Face2) {
		o = q.multiply(0, -_body_length, 0);
		dJointSetHingeAnchor(_motor[0].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
	}
	else if (_enabled == Bodies::Face3) {
		o = q.multiply(_body_width/2, 0, 0);
		dJointSetHingeAnchor(_motor[0].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
	}
	dJointSetHingeAxis(_motor[0].joint, o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[Bodies::Cap], o[0], o[1], o[2]);

	// build rotated joints
	if (_motor[0].theta != 0) this->build_cap(this->getRobotBodyPosition(_enabled, p, q), this->getRobotBodyQuaternion(_enabled, _motor[0].theta, q));

	// set motors
	_motor[0].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[0].id, _body[Bodies::Body], _body[Bodies::Cap]);
	dJointSetAMotorMode(_motor[0].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[0].id, 1);
	dJointSetAMotorAngle(_motor[0].id, 0, 0);
	dJointSetAMotorParam(_motor[0].id, dParamFMax, _motor[0].tau_max);
	dJointSetAMotorParam(_motor[0].id, dParamFudgeFactor, 0.3);
	if (_enabled == Bodies::Face1)
		o = q.multiply(1, 0, 0);
	else if (_enabled == Bodies::Face2)
		o = q.multiply(0, 1, 0);
	else if (_enabled == Bodies::Face3)
		o = q.multiply(-1, 0, 0);
	dJointSetAMotorAxis(_motor[0].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < Bodies::Num_Parts; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

double Dof::calculate_angle(int id) {
	return mod_angle(_motor[0].theta, dJointGetHingeAngle(_motor[0].joint), dJointGetHingeAngleRate(_motor[0].joint)) - _motor[0].offset;
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
	return rs::Vec(_motor[0].theta);
}

void Dof::init_params(void) { }

void Dof::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// store current angle
	_motor[0].theta = calculate_angle(_enabled);
	// set motor angle to current angle
	dJointSetAMotorAngle(_motor[0].id, 0, _motor[0].theta);
	// engage motor depending upon motor mode
	double step = _sim->getStep();
	switch (_motor[0].mode) {
		case SEEK:
			if ((_motor[0].goal - 6*_motor[0].encoder - _motor[0].theta) > rs::Epsilon) {
				_motor[0].state = POSITIVE;
				if (_motor[0].starting++ < 25)
					dJointSetAMotorParam(_motor[0].id, dParamVel, _motor[0].starting*fabs(_motor[0].omega)/50);
				else if (_motor[0].starting++ < 50)
					dJointSetAMotorParam(_motor[0].id, dParamVel, _motor[0].starting*fabs(_motor[0].omega)/150 + 0.3*fabs(_motor[0].omega));
				else if (_motor[0].starting++ < 100)
					dJointSetAMotorParam(_motor[0].id, dParamVel, _motor[0].starting*fabs(_motor[0].omega)/150 + fabs(_motor[0].omega)/3);
				else
					dJointSetAMotorParam(_motor[0].id, dParamVel, fabs(_motor[0].omega));
			}
			else if ((_motor[0].goal - 3*_motor[0].encoder - _motor[0].theta) > rs::Epsilon) {
				_motor[0].state = POSITIVE;
				dJointSetAMotorParam(_motor[0].id, dParamVel, fabs(_motor[0].omega)/2);
			}
			else if ((_motor[0].goal - _motor[0].encoder/2 - _motor[0].theta) > rs::Epsilon) {
				_motor[0].state = POSITIVE;
				dJointSetAMotorParam(_motor[0].id, dParamVel, fabs(_motor[0].omega)/4);
			}
			else if ((_motor[0].theta - _motor[0].goal - 6*_motor[0].encoder) > rs::Epsilon) {
				_motor[0].state = NEGATIVE;
				if (_motor[0].starting++ < 25)
					dJointSetAMotorParam(_motor[0].id, dParamVel, -_motor[0].starting*fabs(_motor[0].omega)/50);
				else if (_motor[0].starting++ < 50)
					dJointSetAMotorParam(_motor[0].id, dParamVel, -_motor[0].starting*fabs(_motor[0].omega)/150 - 0.3*fabs(_motor[0].omega));
				else if (_motor[0].starting++ < 100)
					dJointSetAMotorParam(_motor[0].id, dParamVel, -_motor[0].starting*fabs(_motor[0].omega)/150 - fabs(_motor[0].omega)/3);
				else
					dJointSetAMotorParam(_motor[0].id, dParamVel, -fabs(_motor[0].omega));
			}
			else if ((_motor[0].theta - _motor[0].goal - 3*_motor[0].encoder) > rs::Epsilon) {
				_motor[0].state = NEGATIVE;
				dJointSetAMotorParam(_motor[0].id, dParamVel, -fabs(_motor[0].omega)/2);
			}
			else if ((_motor[0].theta - _motor[0].goal - _motor[0].encoder/2) > rs::Epsilon) {
				_motor[0].state = NEGATIVE;
				dJointSetAMotorParam(_motor[0].id, dParamVel, -fabs(_motor[0].omega)/4);
			}
			else {
				_motor[0].state = HOLD;
				_motor[0].starting = 0;
				dJointSetAMotorParam(_motor[0].id, dParamVel, 0);
			}
			break;
		case SINGULAR:
			// reenable body on start
			dBodyEnable(_body[0]);

			// set new omega
			_motor[0].goal = _values[_loc++];
			_motor[0].omega = (_motor[0].goal - _motor[0].theta)/step;
			_motor[0].state = NEUTRAL;

			// move forever
			dJointEnable(_motor[0].id);
			dJointSetAMotorParam(_motor[0].id, dParamVel, _motor[0].omega);

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
	MUTEX_LOCK(&_motor[0].success_mutex);
	// zero velocity == stopped
	_motor[0].stopping += (!(int)(dJointGetAMotorParam(_motor[0].id, dParamVel)*1000) );
	// once motor has been stopped for 10 steps
	if (_motor[0].stopping == 50) {
		_motor[0].stopping = 0;
		_motor[0].success = 1;
	}
	// signal success
	if (_motor[0].success)
		COND_SIGNAL(&_motor[0].success_cond);
	// unlock mutex
	MUTEX_UNLOCK(&_motor[0].success_mutex);

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

