#include <iostream>

#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Mindstorms>

using namespace rsSim;
using namespace rsMindstorms;

Mindstorms::Mindstorms(int form) : rsRobots::Robot(form) {
	// initialize parameters
	this->init_params();
}

Mindstorms::~Mindstorms(void) {
	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}
}

/**********************************************************
	inherited functions
 **********************************************************/
int Mindstorms::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// set wheels
	_wheels[0] = w[0];
	_wheels[1] = w[1];

	// build
	this->buildIndividual(p, q, a);

	// set trackwidth
	const double *pos1, *pos2;
	pos1 = dBodyGetPosition(_body[WHEEL1]);
	pos2 = dBodyGetPosition(_body[WHEEL2]);
	_trackwidth = sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2));

	// success
	return 0;
}

int Mindstorms::buildIndividual(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = rs::D2R(a[i]);
	}

	// build robot bodies
	this->build_body(this->getRobotBodyPosition(BODY, p, q), this->getRobotBodyQuaternion(BODY, 0, q));
	this->build_wheel(WHEEL1, this->getRobotBodyPosition(WHEEL1, p, q), this->getRobotBodyQuaternion(WHEEL1, 0, q));
	this->build_wheel(WHEEL2, this->getRobotBodyPosition(WHEEL2, p, q), this->getRobotBodyQuaternion(WHEEL2, 0, q));

	// joint variable
	std::vector<dJointID> joint;
	joint.resize(_dof);
	rs::Pos o;

	// joint for body to wheel 1
	joint[JOINT1] = dJointCreateHinge(_world, 0);
	dJointAttach(joint[JOINT1], _body[BODY], _body[WHEEL1]);
	o = q.multiply(-_body_width/2, 0, 0);
	dJointSetHingeAnchor(joint[JOINT1], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(joint[JOINT1], o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[WHEEL1], o[0], o[1], o[2]);

    // joint for body to wheel 2
	joint[JOINT2] = dJointCreateHinge(_world, 0);
	dJointAttach(joint[JOINT2], _body[BODY], _body[WHEEL2]);
	o = q.multiply(_body_width/2, 0, 0);
	dJointSetHingeAnchor(joint[JOINT2], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	o = q.multiply(-1, 0, 0);
	dJointSetHingeAxis(joint[JOINT2], o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[WHEEL2], o[0], o[1], o[2]);

	// build motors
	for (int i = 0; i < _dof; i++) {
		_motor[i].id = dJointCreateAMotor(_world, 0);
		_motor[i].joint = joint[i];
		dJointAttach(_motor[i].id, _body[BODY], _body[WHEEL1 + i]);
		dJointSetAMotorMode(_motor[i].id, dAMotorUser);
		dJointSetAMotorNumAxes(_motor[i].id, 1);
		dJointSetAMotorAngle(_motor[i].id, 0, 0);
		dJointSetAMotorParam(_motor[i].id, dParamFMax, _motor[i].tau_max);
		dJointSetAMotorParam(_motor[i].id, dParamFudgeFactor, 0.3);
		dJointDisable(_motor[i].id);
	}
	o = q.multiply(1, 0, 0);
	dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, o[0], o[1], o[2]);
	o = q.multiply(-1, 0, 0);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

double Mindstorms::getAngle(int id) {
	_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_motor[id].joint), dJointGetHingeAngleRate(_motor[id].joint)) - _motor[id].offset;
    return _motor[id].theta;
}

void Mindstorms::init_params(void) {
	_dof = NUM_JOINTS;

	// create arrays for linkbots
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
		_motor[i].omega = 0.7854;			//  45 deg/sec
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
		MUTEX_INIT(&_motor[i].success_mutex);
		COND_INIT(&_motor[i].success_cond);
	}
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

void Mindstorms::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[BODY]);
	// put into accel array
	_accel[0] = R[8];
	_accel[1] = R[9];
	_accel[2] = R[10];
	// add gaussian noise to accel
	this->noisy(_accel, 3, 0.005);

	// update angle values for each degree of freedom
	for (int i = 0; i < _dof; i++) {
		// store current angle
		_motor[i].theta = getAngle(i);
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_motor[i].joint, axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
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
					angle = h*(dt - sin(2*rs::PI*dt)/2/rs::PI) + _motor[i].accel.init;
				else if (_motor[i].mode == ACCEL_HARMONIC)
					angle = h*(1 - cos(rs::PI*dt))/2 + _motor[i].accel.init;

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
				if ((_motor[i].goal - 6*_motor[i].encoder - _motor[i].theta) > rs::EPSILON) {
					_motor[i].state = POSITIVE;
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
				}
				else if ((_motor[i].goal - 3*_motor[i].encoder - _motor[i].theta) > rs::EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].goal - _motor[i].encoder - _motor[i].theta) > rs::EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/4);
				}
				else if ((_motor[i].theta - _motor[i].goal - 6*_motor[i].encoder) > rs::EPSILON) {
					_motor[i].state = NEGATIVE;
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
				}
				else if ((_motor[i].theta - _motor[i].goal - 3*_motor[i].encoder) > rs::EPSILON) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder) > rs::EPSILON) {
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
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void Mindstorms::simPostCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);
	MUTEX_LOCK(&_success_mutex);

	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < _dof; i++) {
		// lock mutex
		MUTEX_LOCK(&_motor[i].success_mutex);
		// zero velocity == stopped
		_motor[i].stopping += (!(int)(dJointGetAMotorParam(this->_motor[i].id, dParamVel)*1000) );
		// once motor has been stopped for 10 steps
		if (_motor[i].stopping == 50) {
			_motor[i].stopping = 0;
			_motor[i].success = 1;
		}
		// signal success
		if (_motor[i].success)
			COND_SIGNAL(&_motor[i].success_cond);
		// unlock mutex
		MUTEX_UNLOCK(&_motor[i].success_mutex);
	}

	// all joints have completed their motions
	if (_motor[JOINT1].success && _motor[JOINT2].success)
		COND_SIGNAL(&_success_cond);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void Mindstorms::build_body(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[BODY], &m);

	// set body parameters
	dBodySetPosition(_body[BODY], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[BODY], Q);
	dBodySetFiniteRotationMode(_body[BODY], 1);

	// build geometries
	dGeomID geom[2];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, _body_width, _body_length, _body_height);
	dGeomSetBody(geom[0], _body[BODY]);
	dGeomSetOffsetPosition(geom[0], 0, 0, 0);

	// set geometry 1 - sphere
	geom[1] = dCreateSphere(_space, 0.010229);
	dGeomSetBody(geom[1], _body[BODY]);
	dGeomSetOffsetPosition(geom[1], 0, -_body_length/2 + 0.010229, -_body_height/2 - 0.010229);
}

void Mindstorms::build_wheel(int id, const rs::Pos &p, const rs::Quat &q) {
	// get radius
	double radius = 0.001;
	if (id == BIG)
		radius = _bigwheel_radius;
	else if (id == SMALL)
		radius = _smallwheel_radius;

	// set mass of body
	dMass m;
	dMassSetCylinder(&m, 270, 1, 2*radius, _wheel_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// set body parameters
	dBodySetPosition(_body[id], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[id], Q);
	dBodySetFiniteRotationMode(_body[id], 1);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, radius, _wheel_depth);
	dGeomSetBody(geom, _body[id]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

