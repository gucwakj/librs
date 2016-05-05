#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Mindstorms>

#include <iostream>

using namespace rsSim;
using namespace rsMindstorms;

Mindstorms::Mindstorms(int form) : rsRobots::Robot(form), rsRobots::Mindstorms(form) {
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
		_motor[i].mode = CONTINUOUS;
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
		_motor[i].tau_max = 6;
		_motor[i].timeout = 0;
		_motor[i].theta = 0;
		RS_MUTEX_INIT(&_motor[i].success_mutex);
		RS_COND_INIT(&_motor[i].success_cond);
	}
}

Mindstorms::~Mindstorms(void) {
	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		RS_MUTEX_DESTROY(&_motor[i].success_mutex);
		RS_COND_DESTROY(&_motor[i].success_cond);
	}
}

/**********************************************************
	protected functions
 **********************************************************/
int Mindstorms::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// init body parts
	for (int i = 0; i < Bodies::Num_Parts; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// set wheels
	this->setWheelLeft(w[0]);
	this->setWheelRight(w[1]);

	// convert input angles to radians
	_motor[Bodies::Joint1].goal = _motor[Bodies::Joint1].theta = 0;
	_motor[Bodies::Joint2].goal = _motor[Bodies::Joint2].theta = rs::D2R(a[0]);
	_motor[Bodies::Joint3].goal = _motor[Bodies::Joint3].theta = rs::D2R(a[1]);
	_motor[Bodies::Joint4].goal = _motor[Bodies::Joint4].theta = 0;

	// build robot bodies
	this->build_body(this->getRobotBodyPosition(Bodies::Body, p, q), this->getRobotBodyQuaternion(Bodies::Body, 0, q));
	this->build_wheel(Bodies::Wheel1, this->getRobotBodyPosition(Bodies::Wheel1, p, q), this->getRobotBodyQuaternion(Bodies::Wheel1, 0, q));
	this->build_wheel(Bodies::Wheel2, this->getRobotBodyPosition(Bodies::Wheel2, p, q), this->getRobotBodyQuaternion(Bodies::Wheel2, 0, q));

	// joint variable
	rs::Pos o;

	// joint for body to wheel 1
	_motor[Bodies::Joint2].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[Bodies::Joint2].joint, _body[Bodies::Body], _body[Bodies::Wheel1]);
	o = q.multiply(this->getWheelDepth()/2, 0, 0);
	o.add(this->getRobotBodyPosition(Bodies::Wheel1, p, q));
	dJointSetHingeAnchor(_motor[Bodies::Joint2].joint, o[0], o[1], o[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(_motor[Bodies::Joint2].joint, o[0], o[1], o[2]);

	// joint for body to wheel 2
	_motor[Bodies::Joint3].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[Bodies::Joint3].joint, _body[Bodies::Body], _body[Bodies::Wheel2]);
	o = q.multiply(-this->getWheelDepth()/2, 0, 0);
	o.add(this->getRobotBodyPosition(Bodies::Wheel2, p, q));
	dJointSetHingeAnchor(_motor[Bodies::Joint3].joint, o[0], o[1], o[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(_motor[Bodies::Joint3].joint, o[0], o[1], o[2]);

	// build dummy motors for empty joints
	_motor[Bodies::Joint1].id = dJointCreateAMotor(_world, 0);
	_motor[Bodies::Joint4].id = dJointCreateAMotor(_world, 0);

	// build motor for wheel 1
	_motor[Bodies::Joint2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[Bodies::Joint2].id, _body[Bodies::Body], _body[Bodies::Wheel1]);
	dJointSetAMotorMode(_motor[Bodies::Joint2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[Bodies::Joint2].id, 1);
	dJointSetAMotorAngle(_motor[Bodies::Joint2].id, 0, 0);
	dJointSetAMotorParam(_motor[Bodies::Joint2].id, dParamFMax, _motor[Bodies::Joint2].tau_max);
	dJointSetAMotorParam(_motor[Bodies::Joint2].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[Bodies::Joint2].id);
	o = q.multiply(1, 0, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint2].id, 0, 1, o[0], o[1], o[2]);

	// build motor for wheel 2
	_motor[Bodies::Joint3].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[Bodies::Joint3].id, _body[Bodies::Body], _body[Bodies::Wheel2]);
	dJointSetAMotorMode(_motor[Bodies::Joint3].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[Bodies::Joint3].id, 1);
	dJointSetAMotorAngle(_motor[Bodies::Joint3].id, 0, 0);
	dJointSetAMotorParam(_motor[Bodies::Joint3].id, dParamFMax, _motor[Bodies::Joint3].tau_max);
	dJointSetAMotorParam(_motor[Bodies::Joint3].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[Bodies::Joint3].id);
	o = q.multiply(1, 0, 0);
	dJointSetAMotorAxis(_motor[Bodies::Joint3].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < Bodies::Num_Parts; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// set trackwidth
	const double *pos1, *pos2;
	pos1 = dBodyGetPosition(_body[Bodies::Wheel1]);
	pos2 = dBodyGetPosition(_body[Bodies::Wheel2]);
	_trackwidth = sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2));

	// success
	return 0;
}

void Mindstorms::simPreCollisionThread(void) {
	// lock angle and goal
	RS_MUTEX_LOCK(&_goal_mutex);
	RS_MUTEX_LOCK(&_theta_mutex);

	// calculate acceleration
	//  a = (v_new - v_old)/time
	if (_sim->getClock() > 0.1) {
		const dReal *v = dBodyGetLinearVel(_body[Bodies::Body]);
		for (int i = 0; i < 3; i++) {
			_a[i] = (v[i] - _v[i])/_sim->getStep();
			_v[i] = v[i];
		}
	}

	// update angle values for each degree of freedom
	for (int i = 0; i < _dof; i++) {
		// skip disabled joints
		if (i == Bodies::Joint1 || i == Bodies::Joint4) continue;
		// store current angle
		_motor[i].theta = this->mod_angle(_motor[i].theta, dJointGetHingeAngle(_motor[i].joint), dJointGetHingeAngleRate(_motor[i].joint)) - _motor[i].offset;
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_motor[i].joint, axis);
		dBodySetFiniteRotationAxis(_body[i], axis[0], axis[1], axis[2]);
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
				else if ((_motor[i].goal - _motor[i].encoder - _motor[i].theta) > rs::Epsilon) {
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
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder) > rs::Epsilon) {
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

void Mindstorms::simPostCollisionThread(void) {
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
	if (_motor[Bodies::Joint1].success && _motor[Bodies::Joint2].success &&
		_motor[Bodies::Joint3].success && _motor[Bodies::Joint4].success) {
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
void Mindstorms::build_body(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m, m1;
	dMassSetBox(&m, 100, this->getBodyWidth(), this->getBodyLength(), this->getBodyHeight());
	dMassSetSphere(&m1, 500, 0.010229);
	dMassTranslate(&m1, 0, -this->getBodyLength() + 0.010229 - 0.5, -this->getBodyHeight() / 2 - 0.5);
	dMassAdd(&m, &m1);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[Bodies::Body], &m);

	// set body parameters
	dBodySetPosition(_body[Bodies::Body], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[Bodies::Body], Q);

	// build geometries
	dGeomID geom[2];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, this->getBodyWidth(), this->getBodyLength(), this->getBodyHeight());
	dGeomSetBody(geom[0], _body[Bodies::Body]);
	dGeomSetOffsetPosition(geom[0], 0, 0, 0);

	// set geometry 1 - sphere
	// FAKED: adjust caster to make 3ds model look good
	double offset;
	if (this->getWheelLeft() == Connectors::Big && this->getWheelRight() == Connectors::Big)
		offset = -0.006;
	else if (this->getWheelLeft() == Connectors::Small && this->getWheelRight() == Connectors::Big)
		offset = -0.003;
	else if (this->getWheelLeft() == Connectors::Big && this->getWheelRight() == Connectors::Small)
		offset = -0.003;
	else
		offset = 0.001;
	geom[1] = dCreateSphere(_space, 0.010229);
	dGeomSetBody(geom[1], _body[Bodies::Body]);
	dGeomSetOffsetPosition(geom[1], 0, -this->getBodyLength() / 2 + 0.010229, -this->getBodyHeight() / 2 + offset);
}

void Mindstorms::build_wheel(int id, const rs::Pos &p, const rs::Quat &q) {
	// get radius
	double radius = 0.001;
	if (this->getWheel(id-1) == Connectors::Big)
		radius = this->getBigWheelRadius();
	else if (this->getWheel(id-1) == Connectors::Small)
		radius = this->getSmallWheelRadius();

	// set mass of body
	dMass m;
	dMassSetCylinder(&m, 200, 1, radius, this->getWheelDepth());
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// set body parameters
	dBodySetPosition(_body[id], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[id], Q);
	dBodySetFiniteRotationMode(_body[id], 1);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, radius, this->getWheelDepth());
	dGeomSetBody(geom, _body[id]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

