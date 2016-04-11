#include <iostream>
#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Linkbot>

using namespace rsSim;
using namespace rsLinkbot;

Linkbot::Linkbot(int form) : rsRobots::Robot(form), rsRobots::Linkbot(form) {
	// number of degrees of freedom
	_dof = Bodies::Num_Joints;

	// create arrays of proper size
	_motor.resize(_dof);

	// fill with default data
	for (int i = 0; i < _dof; i++) {
		_motor[i].accel.init = 0;
		_motor[i].accel.run = 0;
		_motor[i].accel.period = 0;
		_motor[i].accel.start = 0;
		_motor[i].alpha = 0;
		_motor[i].encoder = rs::D2R(0.25);
		_motor[i].goal = 0;
		_motor[i].mode = SEEK;
		_motor[i].offset = 0;
		_motor[i].omega = 1.5708;			// 90 deg/sec
		_motor[i].omega_max = 4.1888;		// 240 deg/sec
		_motor[i].record = false;
		_motor[i].record_active = false;
		_motor[i].record_num = 0;
		_motor[i].safety_angle = 10;
		_motor[i].safety_timeout = 4;
		_motor[i].starting = 0;
		_motor[i].state = NEUTRAL;
		_motor[i].stopping = 0;
		_motor[i].success = true;
		_motor[i].tau_max = 2;
		_motor[i].timeout = 0;
		_motor[i].theta = 0;
		RS_MUTEX_INIT(&_motor[i].success_mutex);
		RS_COND_INIT(&_motor[i].success_cond);
	}
}

Linkbot::~Linkbot(void) {
	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		RS_MUTEX_DESTROY(&_motor[i].success_mutex);
		RS_COND_DESTROY(&_motor[i].success_cond);
	}
}

/**********************************************************
	public functions
 **********************************************************/
int Linkbot::addConnector(int type, int face, int orientation, double size, int side, int conn, int orientation2) {
	// get connector body position
	rs::Pos P1 = this->getRobotFacePosition(face, this->getPosition(), this->getQuaternion());
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, _motor[face-1].theta, this->getQuaternion());
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
		case Connectors::BigWheel:
			this->build_wheel(_conn.back(), this->getBigWheelRadius());
			break;
		case Connectors::Bridge:
			this->build_bridge(_conn.back());
			break;
		case Connectors::Caster:
			this->build_caster(_conn.back(), static_cast<int>(size));
			break;
		case Connectors::Cube:
			this->build_cube(_conn.back());
			break;
		case Connectors::Faceplate:
			this->build_faceplate(_conn.back());
			break;
		case Connectors::Gripper:
			this->build_gripper(_conn.back(), face);
			break;
		case Connectors::Omniplate:
			this->build_omnidrive(_conn.back());
			break;
		case Connectors::Simple:
			this->build_simple(_conn.back());
			break;
		case Connectors::SmallWheel:
			this->build_wheel(_conn.back(), this->getSmallWheelRadius());
			break;
		case Connectors::TinyWheel:
			this->build_wheel(_conn.back(), this->getTinyWheelRadius());
			break;
		case Connectors::Wheel:
			this->build_wheel(_conn.back(), size);
			break;
	}

	// set body parameters
	dBodySetPosition(_conn.back().body, P1[0], P1[1], P1[2]);
	dQuaternion Q = {Q1[3], Q1[0], Q1[1], Q1[2]};
	dBodySetQuaternion(_conn.back().body, Q);

	// fix connector to body
	this->fixConnectorToBody(face, _conn.back().body, conn);

	// success
	return 0;
}

void Linkbot::calculateTrackwidth(void) {
	double wheel[4] = {0};
	const double *pos;
	int j = 0;
	for (unsigned int i = 0; i < _conn.size(); i++) {
		switch (_conn[i].type) {
			case Connectors::BigWheel:
			case Connectors::SmallWheel:
			case Connectors::TinyWheel:
			case Connectors::Wheel:
				pos = dBodyGetPosition(_conn[i].body);
				wheel[j++] = pos[0];
				wheel[j++] = pos[1];
				break;
			default:
				break;
		}
	}
	_trackwidth = sqrt(pow(wheel[0] - wheel[2], 2) + pow(wheel[1] - wheel[3], 2));
	if (fabs(_trackwidth) < rs::Epsilon) _trackwidth = 0.094;
}

const rs::Pos Linkbot::getCoM(double &mass) {
	double total = 0;
	double x = 0, y = 0, z = 0;
	// body parts
	for (int i = 0; i < Bodies::Num_Parts; i++) {
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

const rs::Vec Linkbot::getJoints(void) {
	return rs::Vec(_motor[Bodies::Joint1].theta, _motor[Bodies::Joint2].theta, _motor[Bodies::Joint3].theta);
}

/**********************************************************
	protected functions
 **********************************************************/
int Linkbot::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// build
	this->build_robot(p, q, a);

	// set wheels
	this->setWheelLeft(w[0]);
	this->setWheelRight(w[1]);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int Linkbot::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, dBodyID base, int face, int orientation, int ground) {
	// calculate center of robot location
	rs::Pos P = this->getRobotCenterPosition(face, p, q);
	rs::Quat Q = this->getRobotCenterQuaternion(face, orientation, rs::D2R(a[face-1]), q);

	// build new module
	this->build_robot(P, Q, a);

	// add fixed joint to attach two modules
	this->fixBodyToConnector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

void Linkbot::simPreCollisionThread(void) {
	// lock angle and goal
	RS_MUTEX_LOCK(&_goal_mutex);
	RS_MUTEX_LOCK(&_theta_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[Bodies::Body]);
	// put into accel array
	_accel[0] = R[8];
	_accel[1] = R[9];
	_accel[2] = R[10];
	// add gaussian noise to accel
	//this->noisy(_accel, 3, 0.005);

	// update angle values for each degree of freedom
	for (int i = 0; i < _dof; i++) {
		if (this->getDisabled() == i) continue;
		// store current angle
		_motor[i].theta = calculate_angle(i);
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_motor[i].joint, axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
		for (unsigned int k = 0; k < _conn.size(); k++) {
			if (_conn[k].face == i+1)
				dBodySetFiniteRotationAxis(_conn[k].body, axis[0], axis[1], axis[2]);
		}
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
		// engage motor depending upon motor mode
		double t = 0, angle = 0, h = 0, dt = 0;
		double step = _sim->getStep();
		switch (_motor[i].mode) {
			case ACCEL_CONSTANT:
				// check if done with acceleration
				if (_motor[i].timeout) {
					_motor[i].timeout--;
				}
				else {
					_motor[i].mode = CONTINUOUS;
					if (_motor[i].omega > 0) _motor[i].state = POSITIVE;
					else if (_motor[i].omega < 0) _motor[i].state = NEGATIVE;
					_motor[i].timeout = -1;
				}

				// set new theta
				_motor[i].goal += step*_motor[i].omega;
				if (_motor[i].omega <= _motor[i].omega_max) {
					_motor[i].goal += _motor[i].alpha*step*step/2;
				}

				// move to new theta
				dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].omega);

				// update omega
				_motor[i].omega += step * _motor[i].alpha;
				if (_motor[i].omega > _motor[i].omega_max)
					_motor[i].omega = _motor[i].omega_max;
				else if (_motor[i].omega < -_motor[i].omega_max)
					_motor[i].omega = -_motor[i].omega_max;
				break;
			case ACCEL_CYCLOIDAL:
			case ACCEL_HARMONIC:
				// init params on first run
				if (_motor[i].accel.run == 0) {
					_motor[i].accel.init = _motor[i].theta;
					_motor[i].accel.start = _sim->getClock();
					_motor[i].accel.run = 1;
					break;
				}

				// calculate new angle
				h = _motor[i].goal - _motor[i].accel.init;
				t = _sim->getClock();
				dt = (t - _motor[i].accel.start)/_motor[i].accel.period;
				if (_motor[i].mode == ACCEL_CYCLOIDAL)
					angle = h*(dt - sin(2*rs::Pi*dt)/2/rs::Pi) + _motor[i].accel.init;
				else if (_motor[i].mode == ACCEL_HARMONIC)
					angle = h*(1 - cos(rs::Pi*dt))/2 + _motor[i].accel.init;

				// set new omega
				_motor[i].omega = (angle - _motor[i].theta)/step;

				// give it an initial push
				if (0 < _motor[i].omega && _motor[i].omega < 0.01)
					_motor[i].omega = 0.01;
				else if (-0.01 < _motor[i].omega && _motor[i].omega < 0)
					_motor[i].omega = -0.01;

				// move until timeout is reached
				if (_motor[i].timeout) {
					dJointEnable(_motor[i].id);
					dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].omega);
					_motor[i].timeout--;
				}
				else {
					_motor[i].mode = CONTINUOUS;
					if (_motor[i].omega > 0) _motor[i].state = POSITIVE;
					else if (_motor[i].omega < 0) _motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
					_motor[i].accel.run = 0;
					_motor[i].timeout = -1;
				}
				break;
			case CONTINUOUS:
				switch (_motor[i].state) {
					case POSITIVE:
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
						break;
					case NEGATIVE:
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
						break;
					case HOLD:
						dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
						break;
					case NEUTRAL:
						dJointDisable(_motor[i].id);
						break;
				}
				break;
			case SEEK:
				if ((_motor[i].goal - 6*_motor[i].encoder - _motor[i].theta) > rs::Epsilon) {
					_motor[i].state = POSITIVE;
					if (_motor[i].starting < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
					_motor[i].starting++;
				}
				else if ((_motor[i].goal - 3*_motor[i].encoder - _motor[i].theta) > rs::Epsilon) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].goal - _motor[i].encoder/2 - _motor[i].theta) > rs::Epsilon) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/4);
				}
				else if ((_motor[i].theta - _motor[i].goal - 6*_motor[i].encoder) > rs::Epsilon) {
					_motor[i].state = NEGATIVE;
					if (_motor[i].starting < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
					_motor[i].starting++;
				}
				else if ((_motor[i].theta - _motor[i].goal - 3*_motor[i].encoder) > rs::Epsilon) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder/2) > rs::Epsilon) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/4);
				}
				else {
					_motor[i].state = HOLD;
					_motor[i].starting = 0;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
				}
				break;
		}
	}

	// unlock angle and goal
	RS_MUTEX_UNLOCK(&_theta_mutex);
	RS_MUTEX_UNLOCK(&_goal_mutex);
}

void Linkbot::simPostCollisionThread(void) {
	// lock angle and goal
	RS_MUTEX_LOCK(&_goal_mutex);
	RS_MUTEX_LOCK(&_theta_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < _dof; i++) {
		// lock mutex
		RS_MUTEX_LOCK(&_motor[i].success_mutex);
		// zero velocity == stopped
		_motor[i].stopping += (!(int)(dJointGetAMotorParam(this->_motor[i].id, dParamVel)*1000) );
		// once motor has been stopped for 10 steps
		if (_motor[i].stopping == 50) {
			_motor[i].stopping = 0;
			_motor[i].success = 1;
			RS_COND_SIGNAL(&_motor[i].success_cond);
		}
		// unlock mutex
		RS_MUTEX_UNLOCK(&_motor[i].success_mutex);
	}

	// all joints have completed their motions
	RS_MUTEX_LOCK(&_success_mutex);
	if (_motor[Bodies::Joint1].success && _motor[Bodies::Joint2].success && _motor[Bodies::Joint3].success) {
		_success = true;
		RS_COND_SIGNAL(&_success_cond);
	}
	RS_MUTEX_UNLOCK(&_success_mutex);

	// unlock angle and goal
	RS_MUTEX_UNLOCK(&_theta_mutex);
	RS_MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void Linkbot::build_body(const rs::Pos &p, const rs::Quat &q) {
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

void Linkbot::build_bridge(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), this->getBridgeLength(), this->getConnHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), this->getBridgeLength(), this->getConnHeight());
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_cap(int id, const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m;
	if (id == 2)
		dMassSetCylinder(&m, 270, 2, 2*this->getFaceRadius(), this->getFaceDepth());
	else
		dMassSetCylinder(&m, 270, 1, 2*this->getFaceRadius(), this->getFaceDepth());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// set body parameters
	dBodySetPosition(_body[id], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[id], Q);
	dBodySetFiniteRotationMode(_body[id], 1);

	// destroy old geom when rotating joint on rebuild
	dGeomID geom1 = dBodyGetFirstGeom(_body[id]);
	if (geom1) dGeomDestroy(geom1);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, this->getFaceRadius(), this->getFaceDepth());
	dGeomSetBody(geom, _body[id]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

void Linkbot::build_caster(Connector &conn, int custom) {
	// set mass of body
	dMass m, m1;
	dMassSetBox(&m, 2000, 5*this->getConnDepth(), 1.5*this->getFaceRadius(), this->getBodyHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// build geometries
	dGeomID geom[4];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, this->getConnDepth(), 1.5*this->getFaceRadius(), this->getBodyHeight());
	dGeomSetBody(geom[0], conn.body);

	// default 3ds caster
	if (!custom) {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn.body);
		dGeomSetOffsetPosition(geom[1], this->getConnDepth()/2 + 0.01 - m.c[0], -m.c[1], -this->getBodyHeight()/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, 0.003);
		dGeomSetBody(geom[2], conn.body);
		dGeomSetOffsetPosition(geom[2], this->getConnDepth()/2 + 0.0368 - m.c[0], -m.c[1], -this->getBodyHeight()/2 + 0.0001 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn.body);
		dGeomSetOffsetPosition(geom[3], this->getConnDepth()/2 + 0.0368 - m.c[0], -m.c[1], -this->getBodyHeight()/2 - 0.004 - m.c[2]);
	}
	// custom drawn one for mathematics
	else {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn.body);
		dGeomSetOffsetPosition(geom[1], this->getConnDepth()/2 + 0.01, 0, -this->getBodyHeight()/2 + 0.0016);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, this->getWheelRadius() -this->getFaceRadius() - 0.006 + 0.0032);
		dGeomSetBody(geom[2], conn.body);
		dGeomSetOffsetPosition(geom[2], this->getConnDepth()/2 + 0.02, 0, -this->getBodyHeight()/2 - (this->getWheelRadius() -this->getFaceRadius() - 0.006)/2 + 0.0016);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn.body);
		dGeomSetOffsetPosition(geom[3], this->getConnDepth()/2 + 0.02, 0, -this->getBodyHeight()/2 + this->getFaceRadius() - this->getWheelRadius() + 0.006);
	}
}

void Linkbot::build_cube(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 100, this->getCubicLength(), this->getCubicLength(), this->getCubicLength());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getCubicLength(), this->getCubicLength(), this->getCubicLength());
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_faceplate(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), this->getBodyHeight(), this->getBodyHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), this->getBodyHeight(), this->getBodyHeight());
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_gripper(Connector &conn, int face) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), 2*this->getFaceRadius(), this->getConnHeight());

	// build geometries
	dGeomID geom[3];

	// set geometry 0
	geom[0] = dCreateBox(_space, this->getConnDepth(), 4*this->getFaceRadius(), this->getConnHeight()/2);
	dGeomSetBody(geom[0], conn.body);
	dGeomSetOffsetPosition(geom[0], -m.c[0], this->getFaceRadius() - m.c[1], -m.c[2]);

	// set geometry 1
	geom[1] = dCreateBox(_space, 0.062, 0.04, this->getConnDepth());
	dGeomSetBody(geom[1], conn.body);
	dGeomSetOffsetPosition(geom[1], this->getConnDepth()/2 - 0.062/2 - m.c[0], 3*this->getFaceRadius() - 0.02 - m.c[1], this->getConnDepth()/2 + 0.003 - m.c[2]);

	// set geometry 2
	geom[2] = dCreateBox(_space, 0.0344, 0.04, 0.003);
	dGeomSetBody(geom[2], conn.body);
	dGeomSetOffsetPosition(geom[2], this->getConnDepth()/2 - 0.062 + 0.0344/2 - m.c[0], 3*this->getFaceRadius() - 0.02 - m.c[1], -0.003/2 - m.c[2]);

	// center body mass on geoms
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);
}

void Linkbot::build_omnidrive(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getOmniLength(), this->getOmniLength(), this->getConnDepth());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), this->getOmniLength(), this->getOmniLength());
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_robot(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a) {
	// init body parts
	for (int i = 0; i < Bodies::Num_Parts; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = rs::D2R(a[i]);
	}
	_motor[this->getDisabled()].goal = _motor[this->getDisabled()].theta = 0;

	// build robot bodies
	this->build_body(p, q);
	this->build_cap(Bodies::Cap1, this->getRobotBodyPosition(Bodies::Cap1, p, q), this->getRobotBodyQuaternion(Bodies::Cap1, 0, q));
	this->build_cap(Bodies::Cap2, this->getRobotBodyPosition(Bodies::Cap2, p, q), this->getRobotBodyQuaternion(Bodies::Cap2, 0, q));
	this->build_cap(Bodies::Cap3, this->getRobotBodyPosition(Bodies::Cap3, p, q), this->getRobotBodyQuaternion(Bodies::Cap3, 0, q));

	// joint variable
	rs::Pos o;

	// joint for body to cap 1
	_motor[Bodies::Joint1].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[Bodies::Joint1].joint, _body[Bodies::Body], _body[Bodies::Cap1]);
	o = q.multiply(-this->getBodyWidth()/2, 0, 0);
	dJointSetHingeAnchor(_motor[Bodies::Joint1].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(_motor[Bodies::Joint1].joint, o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[Bodies::Cap1], o[0], o[1], o[2]);

	// joint for body to cap 2
	if (this->getDisabled() == Bodies::Joint2) {
		_motor[Bodies::Joint2].joint = dJointCreateFixed(_world, 0);
		dJointAttach(_motor[Bodies::Joint2].joint, _body[Bodies::Body], _body[Bodies::Cap2]);
		dJointSetFixed(_motor[Bodies::Joint2].joint);
	}
	else {
		_motor[Bodies::Joint2].joint = dJointCreateHinge(_world, 0);
		dJointAttach(_motor[Bodies::Joint2].joint, _body[Bodies::Body], _body[Bodies::Cap2]);
		o = q.multiply(0, -this->getBodyLength(), 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint2].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
		dJointSetHingeAxis(_motor[Bodies::Joint2].joint, o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[Bodies::Cap2], o[0], o[1], o[2]);
	}

	// joint for body to cap 3
	if (this->getDisabled() == Bodies::Joint3) {
		_motor[Bodies::Joint3].joint = dJointCreateFixed(_world, 0);
		dJointAttach(_motor[Bodies::Joint3].joint, _body[Bodies::Body], _body[Bodies::Cap3]);
		dJointSetFixed(_motor[Bodies::Joint3].joint);
	}
	else {
		_motor[Bodies::Joint3].joint = dJointCreateHinge(_world, 0);
		dJointAttach(_motor[Bodies::Joint3].joint, _body[Bodies::Body], _body[Bodies::Cap3]);
		o = q.multiply(this->getBodyWidth()/2, 0, 0);
		dJointSetHingeAnchor(_motor[Bodies::Joint3].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
		dJointSetHingeAxis(_motor[Bodies::Joint3].joint, o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[Bodies::Cap3], o[0], o[1], o[2]);
	}

	// build rotated joints
	if (_motor[Bodies::Joint1].theta != 0)
		this->build_cap(Bodies::Cap1, this->getRobotBodyPosition(Bodies::Cap1, p, q), this->getRobotBodyQuaternion(Bodies::Cap1, _motor[Bodies::Joint1].theta, q));
	if (_motor[Bodies::Joint2].theta != 0)
		this->build_cap(Bodies::Cap2, this->getRobotBodyPosition(Bodies::Cap2, p, q), this->getRobotBodyQuaternion(Bodies::Cap2, _motor[Bodies::Joint2].theta, q));
	if (_motor[Bodies::Joint3].theta != 0)
		this->build_cap(Bodies::Cap3, this->getRobotBodyPosition(Bodies::Cap3, p, q), this->getRobotBodyQuaternion(Bodies::Cap3, _motor[Bodies::Joint3].theta, q));

	// build motors
	for (int i = 0; i < _dof; i++) {
		_motor[i].id = dJointCreateAMotor(_world, 0);
		dJointAttach(_motor[i].id, _body[Bodies::Body], _body[Bodies::Cap1 + i]);
		dJointSetAMotorMode(_motor[i].id, dAMotorUser);
		dJointSetAMotorNumAxes(_motor[i].id, 1);
		dJointSetAMotorAngle(_motor[i].id, 0, 0);
		dJointSetAMotorParam(_motor[i].id, dParamFMax, _motor[i].tau_max);
		dJointSetAMotorParam(_motor[i].id, dParamFudgeFactor, 0.3);
	}
	o = q.multiply(1, 0, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint1].id, 0, 1, o[0], o[1], o[2]);
	o = q.multiply(0, 1, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint2].id, 0, 1, o[0], o[1], o[2]);
	o = q.multiply(-1, 0, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint3].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < Bodies::Num_Parts; i++) dBodySetDamping(_body[i], 0.1, 0.1);
}

void Linkbot::build_simple(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, this->getConnDepth(), 2*this->getFaceRadius(), this->getConnHeight());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, this->getConnDepth(), 2*this->getFaceRadius(), this->getConnHeight());
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_wheel(Connector &conn, double size) {
	// store wheel radius
	this->setWheelRadius(size);

	// set mass of body
	dMass m;
	dMassSetCylinder(&m, 170, 1, 2*this->getWheelRadius(), this->getWheelDepth());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, this->getWheelRadius(), this->getWheelDepth());
	dGeomSetBody(geom, conn.body);
	dQuaternion Q = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q);
}

double Linkbot::calculate_angle(int id) {
	if (id == this->getDisabled()) return 0;

	return mod_angle(_motor[id].theta, dJointGetHingeAngle(_motor[id].joint), dJointGetHingeAngleRate(_motor[id].joint)) - _motor[id].offset;
}

