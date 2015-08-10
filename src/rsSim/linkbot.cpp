#include <iostream>
#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Linkbot>

using namespace rsSim;
using namespace rsLinkbot;

Linkbot::Linkbot(int form) : rsRobots::Robot(form), rsRobots::Linkbot(form) {
	// initialize parameters
	this->init_params();
}

Linkbot::~Linkbot(void) {
	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}
}

void Linkbot::moveJointOnce(int id, double *values) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set motion parameters
	_motor[id].mode = SINGULAR;
	_motor[id].state = POSITIVE;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[0]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set array
	_loc[id] = 0;
	_values[id] = values;

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	inherited functions
 **********************************************************/
int Linkbot::addConnector(int type, int face, int orientation, double size, int side, int conn) {
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
		case BIGWHEEL:
			this->build_wheel(_conn.back(), _bigwheel_radius);
			break;
		case BRIDGE:
			this->build_bridge(_conn.back());
			break;
		case CASTER:
			this->build_caster(_conn.back(), static_cast<int>(size));
			break;
		case CUBE:
			this->build_cube(_conn.back());
			break;
		case DOUBLEBRIDGE:
			this->build_doublebridge(_conn.back());
			break;
		case EL:
			this->build_el(_conn.back());
			break;
		case FACEPLATE:
			this->build_faceplate(_conn.back());
			break;
		case FOOT:
			this->build_foot(_conn.back());
			break;
		case GRIPPER:
			this->build_gripper(_conn.back(), face);
			break;
		case OMNIPLATE:
			this->build_omnidrive(_conn.back());
			break;
		case SIMPLE:
			this->build_simple(_conn.back());
			break;
		case SMALLWHEEL:
			this->build_wheel(_conn.back(), _smallwheel_radius);
			break;
		case TINYWHEEL:
			this->build_wheel(_conn.back(), _tinywheel_radius);
			break;
		case WHEEL:
			this->build_wheel(_conn.back(), size);
			break;
	}

	if (type == GRIPPER) {
		_conn.push_back(Connector());
		_conn.back().d_side = -1;
		_conn.back().d_type = -1;
		_conn.back().face = face;
		_conn.back().type = type;
		_conn.back().orientation = orientation;
		this->build_gripper(_conn.back(), 3);
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

int Linkbot::build(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, int ground) {
	// build
	this->buildIndividual(p, q, a);

	// set trackwidth
	/*double wheel[4] = {0};
	const double *pos;
	int j = 0;
	for (int i = 0; i < _conn.size(); i++) {
		switch (_conn[i]->type) {
			case BIGWHEEL:
			case SMALLWHEEL:
			case TINYWHEEL:
			case WHEEL:
				pos = dBodyGetPosition(_conn[i]->body);
				wheel[j++] = pos[0];
				wheel[j++] = pos[1];
				break;
			default:
				break;
		}
	}
	_trackwidth = sqrt(pow(wheel[0] - wheel[2], 2) + pow(wheel[1] - wheel[3], 2));*/
	_trackwidth = 0.094;

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
	this->buildIndividual(P, Q, a);

	// add fixed joint to attach two modules
	this->fix_body_to_connector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int Linkbot::buildIndividual(const rs::Pos &p, const rs::Quat &q, const rs::Vec &a) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = rs::D2R(a[i]);
	}
	_motor[_disabled].goal = _motor[_disabled].theta = 0;

	// build robot bodies
	this->build_body(p, q);
	this->build_face(FACE1, this->getRobotBodyPosition(FACE1, p, q), this->getRobotBodyQuaternion(FACE1, 0, q));
	this->build_face(FACE2, this->getRobotBodyPosition(FACE2, p, q), this->getRobotBodyQuaternion(FACE2, 0, q));
	this->build_face(FACE3, this->getRobotBodyPosition(FACE3, p, q), this->getRobotBodyQuaternion(FACE3, 0, q));

	// joint variable
	rs::Pos o;

	// joint for body to face 1
	_motor[JOINT1].joint = dJointCreateHinge(_world, 0);
	dJointAttach(_motor[JOINT1].joint, _body[BODY], _body[FACE1]);
	o = q.multiply(-_body_width/2, 0, 0);
	dJointSetHingeAnchor(_motor[JOINT1].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(_motor[JOINT1].joint, o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[FACE1], o[0], o[1], o[2]);

	// joint for body to face 2
	if (_disabled == JOINT2) {
		_motor[JOINT2].joint = dJointCreateFixed(_world, 0);
		dJointAttach(_motor[JOINT2].joint, _body[BODY], _body[FACE2]);
		dJointSetFixed(_motor[JOINT2].joint);
	}
	else {
		_motor[JOINT2].joint = dJointCreateHinge(_world, 0);
		dJointAttach(_motor[JOINT2].joint, _body[BODY], _body[FACE2]);
		o = q.multiply(0, -_body_length, 0);
		dJointSetHingeAnchor(_motor[JOINT2].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
		dJointSetHingeAxis(_motor[JOINT2].joint, o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[FACE2], o[0], o[1], o[2]);
	}

	// joint for body to face 3
	if (_disabled == JOINT3) {
		_motor[JOINT3].joint = dJointCreateFixed(_world, 0);
		dJointAttach(_motor[JOINT3].joint, _body[BODY], _body[FACE3]);
		dJointSetFixed(_motor[JOINT3].joint);
	}
	else {
		_motor[JOINT3].joint = dJointCreateHinge(_world, 0);
		dJointAttach(_motor[JOINT3].joint, _body[BODY], _body[FACE3]);
		o = q.multiply(_body_width/2, 0, 0);
		dJointSetHingeAnchor(_motor[JOINT3].joint, o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
		dJointSetHingeAxis(_motor[JOINT3].joint, o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[FACE3], o[0], o[1], o[2]);
	}

	// build rotated joints
	if (_motor[JOINT1].theta != 0)
		this->build_face(FACE1, this->getRobotBodyPosition(FACE1, p, q), this->getRobotBodyQuaternion(FACE1, _motor[JOINT1].theta, q));
	if (_motor[JOINT2].theta != 0)
		this->build_face(FACE2, this->getRobotBodyPosition(FACE2, p, q), this->getRobotBodyQuaternion(FACE2, _motor[JOINT2].theta, q));
	if (_motor[JOINT3].theta != 0)
		this->build_face(FACE3, this->getRobotBodyPosition(FACE3, p, q), this->getRobotBodyQuaternion(FACE3, _motor[JOINT3].theta, q));

	// build motors
	for (int i = 0; i < _dof; i++) {
		_motor[i].id = dJointCreateAMotor(_world, 0);
		dJointAttach(_motor[i].id, _body[BODY], _body[FACE1 + i]);
		dJointSetAMotorMode(_motor[i].id, dAMotorUser);
		dJointSetAMotorNumAxes(_motor[i].id, 1);
		dJointSetAMotorAngle(_motor[i].id, 0, 0);
		dJointSetAMotorParam(_motor[i].id, dParamFMax, _motor[i].tau_max);
		dJointSetAMotorParam(_motor[i].id, dParamFudgeFactor, 0.3);
	}
	o = q.multiply(1, 0, 0);
	dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, o[0], o[1], o[2]);
	o = q.multiply(0, 1, 0);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, o[0], o[1], o[2]);
	o = q.multiply(-1, 0, 0);
	dJointSetAMotorAxis(_motor[JOINT3].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

double Linkbot::calculate_angle(int id) {
	if (id == _disabled)
		return 0;

	return mod_angle(_motor[id].theta, dJointGetHingeAngle(_motor[id].joint), dJointGetHingeAngleRate(_motor[id].joint)) - _motor[id].offset;
}

const rs::Pos Linkbot::getCoM(double &mass) {
	double total = 0;
	double x = 0, y = 0, z = 0;
	// body parts
	for (int i = 0; i < NUM_PARTS; i++) {
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
	return rs::Vec(_motor[JOINT1].theta, _motor[JOINT2].theta, _motor[JOINT3].theta);
}

void Linkbot::init_params(void) {
	_dof = NUM_JOINTS;

	// create arrays for linkbots
	_motor.resize(_dof);
	_values.resize(_dof);	// research

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
		_motor[i].omega = 0.7854;			// 45 deg/sec
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
		// research: array of values
		_values[i] = NULL;
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

void Linkbot::simPreCollisionThread(void) {
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
	//this->noisy(_accel, 3, 0.005);

	// update angle values for each degree of freedom
	for (int i = 0; i < _dof; i++) {
		if (_disabled == i) continue;
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
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].starting*fabs(_motor[i].omega)/150 + fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
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
					if (_motor[i].starting++ < 25)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/50);
					else if (_motor[i].starting++ < 50)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - 0.3*fabs(_motor[i].omega));
					else if (_motor[i].starting++ < 100)
						dJointSetAMotorParam(_motor[i].id, dParamVel, -_motor[i].starting*fabs(_motor[i].omega)/150 - fabs(_motor[i].omega)/3);
					else
						dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
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
			case SINGULAR: {
				// reenable body on start
				dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
				dBodyEnable(_body[0]);

				// set new omega
				double angle = _values[i][_loc[i]];
				_motor[i].omega = (angle - _motor[i].theta)/step;
				_motor[i].goal = angle;
				_motor[i].state = NEUTRAL;
				_loc[i]++;

				// move forever
				dJointEnable(_motor[i].id);
				dJointSetAMotorParam(_motor[i].id, dParamVel, _motor[i].omega);

				// end
				break;
			}
		}
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void Linkbot::simPostCollisionThread(void) {
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
	if (_motor[JOINT1].success && _motor[JOINT2].success && _motor[JOINT3].success)
		COND_SIGNAL(&_success_cond);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void Linkbot::build_body(const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m, m1;
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, 0, -_body_length/2, 0);
	dMassSetCylinder(&m1, 400, 1, _body_radius, _body_width);
	dMassAdd(&m, &m1);
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
	dGeomSetOffsetPosition(geom[0], 0, -_body_length/2, 0);

	// set geometry 1 - cylinder
	geom[1] = dCreateCylinder(_space, _body_radius, _body_width);
	dGeomSetBody(geom[1], _body[BODY]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom[1], Q1);
}

void Linkbot::build_bridge(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, _bridge_length, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, _bridge_length, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_caster(Connector &conn, int custom) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 1000, 2*_conn_depth, 1.5*_face_radius, _body_height);
	//dMassTranslate(&m, 8*_conn_depth, 0, -_body_height/2);

	// build geometries
	dGeomID geom[4];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, _conn_depth, 1.5*_face_radius, _body_height);
	dGeomSetBody(geom[0], conn.body);
	dGeomSetOffsetPosition(geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// default 3ds caster
	if (!custom) {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn.body);
		dGeomSetOffsetPosition(geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, 0.003);
		dGeomSetBody(geom[2], conn.body);
		dGeomSetOffsetPosition(geom[2], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 + 0.0001 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn.body);
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 - 0.004 - m.c[2]);
	}
	// custom drawn one for mathematics
	else {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn.body);
		dGeomSetOffsetPosition(geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, _wheel_radius -_face_radius - 0.006 + 0.0032);
		dGeomSetBody(geom[2], conn.body);
		dGeomSetOffsetPosition(geom[2], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 - (_wheel_radius -_face_radius - 0.006)/2 + 0.0016 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn.body);
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 + _face_radius - _wheel_radius + 0.006 - m.c[2]);
	}

	// center body mass on geoms
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);
}

void Linkbot::build_cube(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 100, _cubic_length, _cubic_length, _cubic_length);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _cubic_length, _cubic_length, _cubic_length);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_doublebridge(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, 2*_conn_depth, _bridge_length, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, 2*_conn_depth, _bridge_length, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_el(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_face(int id, const rs::Pos &p, const rs::Quat &q) {
	// set mass of body
	dMass m;
	if (id == 2)
		dMassSetCylinder(&m, 270, 2, 2*_face_radius, _face_depth);
	else
		dMassSetCylinder(&m, 270, 1, 2*_face_radius, _face_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// set body parameters
	dBodySetPosition(_body[id], p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(_body[id], Q);
	dBodySetFiniteRotationMode(_body[id], 1);

	// destroy old geom when rotating joint on rebuild
	dGeomID geom1 = dBodyGetFirstGeom(_body[id]);
	while (geom1) {
		dGeomDestroy(geom1);
		geom1 = dBodyGetNextGeom(geom1);
	}

	// set geometry
	dGeomID geom = dCreateCylinder(_space, _face_radius, _face_depth);
	dGeomSetBody(geom, _body[id]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q1);
}

void Linkbot::build_faceplate(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, _body_height, _body_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, _body_height, _body_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_foot(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_gripper(Connector &conn, int face) {
	// create body
	int i = (face == 1) ? 1 : -1;

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);

	// build geometries
	dGeomID geom[3];

	// set geometry 0
	geom[0] = dCreateBox(_space, _conn_depth, 4*_face_radius, _conn_height/2);
	dGeomSetBody(geom[0], conn.body);
	dGeomSetOffsetPosition(geom[0], -m.c[0], -i*_face_radius - m.c[1], -m.c[2]);

	// set geometry 1
	geom[1] = dCreateBox(_space, 0.062, 0.04, _conn_depth);
	dGeomSetBody(geom[1], conn.body);
	dGeomSetOffsetPosition(geom[1], _conn_depth/2 - 0.062/2 - m.c[0], -i*3*_face_radius + i*0.02 - m.c[1], i*_conn_height/4 - i*_conn_depth/2 - m.c[2]);

	// set geometry 2
	geom[2] = dCreateBox(_space, 0.0344, 0.04, 0.007);
	dGeomSetBody(geom[2], conn.body);
	dGeomSetOffsetPosition(geom[2], _conn_depth/2 - 0.062 + 0.0344/2 - m.c[0], -i*3*_face_radius + i*0.02 - m.c[1], i*_conn_height/4 - i*_conn_depth/2 - i*0.007/2 - m.c[2]);

	// center body mass on geoms
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);
}

void Linkbot::build_omnidrive(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _omni_length, _omni_length, _conn_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, _omni_length, _omni_length);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_simple(Connector &conn) {
	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(geom, conn.body);
}

void Linkbot::build_wheel(Connector &conn, double size) {
	// store wheel radius
	_wheel_radius = size;

	// set mass of body
	dMass m;
	dMassSetCylinder(&m, 170, 1, 2*_wheel_radius, _wheel_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn.body, &m);

	// set geometry
	dGeomID geom = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(geom, conn.body);
	dQuaternion Q = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom, Q);
}

