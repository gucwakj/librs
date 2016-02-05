#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Dof>

using namespace rsSim;
using namespace rsDof;

Dof::Dof(short joint, float scale) : rsRobots::Robot(rs::Dof), rsRobots::Dof(joint, scale) {
	// number of degrees of freedom
	_dof = Bodies::Num_Joints;

	// create arrays of proper size
	_motor.resize(_dof);

	// starting motion
	_start = true;

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

	// init threading
	RS_MUTEX_INIT(&_motor[Bodies::Joint].success_mutex);
	RS_COND_INIT(&_motor[Bodies::Joint].success_cond);
}

Dof::~Dof(void) {
	// delete mutexes
	RS_MUTEX_DESTROY(&_motor[Bodies::Joint].success_mutex);
	RS_COND_DESTROY(&_motor[Bodies::Joint].success_cond);
}

/**********************************************************
	public functions
 **********************************************************/
int Dof::addConnector(int type, int face, int orientation, double size, int side, int conn, int orientation2) {
	// get connector body position
	rs::Pos P1 = this->getRobotFacePosition(face, this->getPosition(), this->getQuaternion());
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, (this->getEnabled() == face) ? _motor[Bodies::Joint].theta : 0, this->getQuaternion());
	if (conn == -1) {
		P1 = this->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = this->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = this->getConnFaceQuaternion(type, side, orientation, Q1);
		P1 = this->getConnBodyPosition(conn, orientation2, P1, Q1);
		Q1 = this->getConnBodyQuaternion(conn, orientation2, Q1);
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
		case Connectors::Plank:
			this->build_plank(_conn.back());
			break;
	}

	// set body parameters
	dBodySetPosition(_conn.back().body, P1[0], P1[1], P1[2]);
	dQuaternion Q = {Q1[3], Q1[0], Q1[1], Q1[2]};
	dBodySetQuaternion(_conn.back().body, Q);

	// fix connector to body
	this->fix_connector_to_body(face, _conn.back().body, conn);

	// success
	return 0;
}

double Dof::getAngle(int id) {
	if (id == this->getEnabled()) return _motor[Bodies::Joint].theta;
	return 0;
}

dBodyID Dof::getBodyID(short face) {
	if (face == this->getEnabled()) return _body[Bodies::Cap];
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

#ifdef RS_RESEARCH
void Dof::moveJointSingular(bool wait) {
	// tell joint whether to wait or not
	_start = wait;

	// set motion parameters
	_motor[Bodies::Joint].mode = SINGULAR;
	_motor[Bodies::Joint].state = POSITIVE;

	// enable motor
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[Bodies::Joint].id);
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, _motor[Bodies::Joint].theta);
	dBodyEnable(_body[0]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	RS_MUTEX_LOCK(&_motor[Bodies::Joint].success_mutex);
	_motor[Bodies::Joint].success = false;
	RS_MUTEX_UNLOCK(&_motor[Bodies::Joint].success_mutex);
}
#endif

/**********************************************************
	protected functions
 **********************************************************/
int Dof::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// build
	this->build_robot(p, q, a);

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
	this->build_robot(P, Q, a);

	// add fixed joint to attach two modules
	this->fix_body_to_connector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

void Dof::simPreCollisionThread(void) {
	// lock angle and goal
	RS_MUTEX_LOCK(&_goal_mutex);
	RS_MUTEX_LOCK(&_theta_mutex);

	// store current angle
	_motor[Bodies::Joint].theta = this->mod_angle(_motor[Bodies::Joint].theta, dJointGetHingeAngle(_motor[Bodies::Joint].joint), dJointGetHingeAngleRate(_motor[Bodies::Joint].joint)) - _motor[Bodies::Joint].offset;
	// set motor angle to current angle
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, _motor[Bodies::Joint].theta);
	// engage motor depending upon motor mode
	double step = _sim->getStep();
#ifdef RS_RESEARCH
	switch (_motor[Bodies::Joint].mode) {
		case SINGULAR:
			// don't move until positive
			if (_start) {
				if (_next_goal > -rs::Epsilon)
					_start = false;
			}
			else {
				// reenable body on start
				dBodyEnable(_body[0]);

				// set new omega
				_motor[Bodies::Joint].goal = _next_goal;
				_motor[Bodies::Joint].omega = (_motor[Bodies::Joint].goal - _motor[Bodies::Joint].theta)/step;
				_motor[Bodies::Joint].state = NEUTRAL;

				// move forever
				dJointEnable(_motor[Bodies::Joint].id);
				dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamVel, _motor[Bodies::Joint].omega);
			}

			// end
			break;
	}
#endif

	// unlock angle and goal
	RS_MUTEX_UNLOCK(&_theta_mutex);
	RS_MUTEX_UNLOCK(&_goal_mutex);
}

void Dof::simPostCollisionThread(void) {
	// lock angle and goal
	RS_MUTEX_LOCK(&_goal_mutex);
	RS_MUTEX_LOCK(&_theta_mutex);
	RS_MUTEX_LOCK(&_success_mutex);
	RS_MUTEX_LOCK(&_motor[Bodies::Joint].success_mutex);

	// zero velocity == stopped
	_motor[Bodies::Joint].stopping += (!(int)(dJointGetAMotorParam(_motor[Bodies::Joint].id, dParamVel)*1000) );
	// once motor has been stopped for 10 steps
	if (_motor[Bodies::Joint].stopping == 50) {
		_motor[Bodies::Joint].stopping = 0;
		_motor[Bodies::Joint].success = 1;
		RS_COND_SIGNAL(&_motor[Bodies::Joint].success_cond);
		_success = true;
		RS_COND_SIGNAL(&_success_cond);
	}

	// unlock mutex
	RS_MUTEX_UNLOCK(&_motor[Bodies::Joint].success_mutex);
	RS_MUTEX_UNLOCK(&_success_mutex);
	RS_MUTEX_UNLOCK(&_theta_mutex);
	RS_MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void Dof::build_body(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m, m1;
	dMassSetBox(&m, 1000, this->getBodyWidth(), this->getBodyLength(), this->getBodyHeight());
	dMassTranslate(&m, 0, -this->getBodyLength()/2, 0);
	dMassSetCylinder(&m1, 400, 1, this->getBodyRadius(), this->getBodyWidth());
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
	geom[0] = dCreateBox(_space, this->getBodyWidth(), this->getBodyLength(), this->getBodyHeight());
	dGeomSetBody(geom[0], _body[Bodies::Body]);
	dGeomSetOffsetPosition(geom[0], 0, -this->getBodyLength()/2, 0);

	// set geometry 1 - cylinder
	geom[1] = dCreateCylinder(_space, this->getBodyRadius(), this->getBodyWidth());
	dGeomSetBody(geom[1], _body[Bodies::Body]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom[1], Q1);
}

void Dof::build_cap(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m;
	if (this->getEnabled() == Bodies::Face2)
		dMassSetCylinder(&m, 270, 2, 2*this->getCapRadius(), this->getCapDepth());
	else
		dMassSetCylinder(&m, 270, 1, 2*this->getCapRadius(), this->getCapDepth());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[Bodies::Cap], &m);

	// set body parameters
	dBodySetPosition(_body[Bodies::Cap], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[Bodies::Cap], Q);
	dBodySetFiniteRotationMode(_body[Bodies::Cap], 1);

	// destroy old geom when rotating joint on rebuild
	dGeomID geom1 = dBodyGetFirstGeom(_body[Bodies::Cap]);
	if (geom1) dGeomDestroy(geom1);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, this->getCapRadius(), this->getCapDepth());
	dGeomSetBody(geom, _body[Bodies::Cap]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

void Dof::build_el(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), 2*this->getCapRadius(), this->getConnHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), 2*this->getCapRadius(), this->getConnHeight());
	dGeomSetBody(geom, conn.body);
}

void Dof::build_foot(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), 2*this->getCapRadius(), this->getConnHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), 2*this->getCapRadius(), this->getConnHeight());
	dGeomSetBody(geom, conn.body);
}

void Dof::build_plank(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), this->getElLength(), this->getConnHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), this->getElLength(), this->getConnHeight());
	dGeomSetBody(geom, conn.body);
}

void Dof::build_robot(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a) {
	// init body parts
	for (int i = 0; i < Bodies::Num_Parts; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angle to radians
	_motor[Bodies::Joint].goal = _motor[Bodies::Joint].theta = rs::D2R(a[0]);

	// build robot bodies
	this->build_body(p, q);
	this->build_cap(this->getRobotBodyPosition(this->getEnabled(), p, q), this->getRobotBodyQuaternion(this->getEnabled(), 0, q));

	// build joints
	rs::Pos o;
	_motor[Bodies::Joint].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[Bodies::Joint].joint, _body[Bodies::Body], _body[Bodies::Cap]);
	if (this->getEnabled() == Bodies::Face1) {
		o = q.multiply(-this->getBodyWidth()/2, 0, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(1, 0, 0);
	}
	else if (this->getEnabled() == Bodies::Face2) {
		o = q.multiply(0, -this->getBodyLength(), 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
	}
	else if (this->getEnabled() == Bodies::Face3) {
		o = q.multiply(this->getBodyWidth()/2, 0, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
	}
	dJointSetHingeAxis(_motor[Bodies::Joint].joint, o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[Bodies::Cap], o[0], o[1], o[2]);

	// build rotated joints
	if (_motor[Bodies::Joint].theta != 0) this->build_cap(this->getRobotBodyPosition(this->getEnabled(), p, q), this->getRobotBodyQuaternion(this->getEnabled(), _motor[Bodies::Joint].theta, q));

	// set motors
	_motor[Bodies::Joint].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[Bodies::Joint].id, _body[Bodies::Body], _body[Bodies::Cap]);
	dJointSetAMotorMode(_motor[Bodies::Joint].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[Bodies::Joint].id, 1);
	dJointSetAMotorAngle(_motor[Bodies::Joint].id, 0, 0);
	dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamFMax, _motor[Bodies::Joint].tau_max);
	dJointSetAMotorParam(_motor[Bodies::Joint].id, dParamFudgeFactor, 0.3);
	dJointSetAMotorAxis(_motor[Bodies::Joint].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < Bodies::Num_Parts; i++) dBodySetDamping(_body[i], 0.1, 0.1);
}

