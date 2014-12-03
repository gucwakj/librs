#include "nxt.hpp"

CNXT::CNXT(void) : Robot(JOINT1, JOINT2) {
	// initialize parameters
	this->initParams(0, NXT);

	// initialize dimensions
	this->initDims();
}

CNXT::~CNXT(void) {
	// remove robot from simulation
	if ( _sim != NULL && !(_sim->deleteRobot(_pos)) )
		delete _sim;

	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}

	// destroy geoms
	if (_connected) {
		for (int i = NUM_PARTS - 1; i >= 0; i--) { delete [] _geom[i]; }
	}
}

int CNXT::getJointAngles(double &angle1, double &angle2, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);

	// success
	return 0;
}

int CNXT::getJointAnglesInstant(double &angle1, double &angle2) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);

	// success
	return 0;
}

int CNXT::getJointSpeeds(double &speed1, double &speed2) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);

	// success
	return 0;
}

int CNXT::getJointSpeedRatios(double &ratio1, double &ratio2) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;

	// success
	return 0;
}

int CNXT::move(double angle1, double angle2) {
	this->moveNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::moveNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CNXT::moveTo(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::moveToNB(double angle1, double angle2) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CNXT::moveToByTrackPos(double angle1, double angle2) {
	this->moveToByTrackPosNB(angle1, angle2);
	this->moveWait();

	// success
	return 0;
}

int CNXT::moveToByTrackPosNB(double angle1, double angle2) {
	this->moveToNB(angle1, angle2);

	// success
	return 0;
}

int CNXT::recordAngles(double *time, double *angle1, double *angle2, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CNXT::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CNXT::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CNXT::setJointSpeeds(double speed1, double speed2) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);

	// success
	return 0;
}

int CNXT::setJointSpeedRatios(double ratio1, double ratio2) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CNXT::build(XMLRobot *robot, int really) {
	// adjust height for wheels
	robot->z += (_wheel_radius - _body_height/2);
	_radius = _wheel_radius;
	if (fabs(robot->z) > (_body_radius-EPSILON)) { robot->z += _body_height/2; }

	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta,	-cphi*stheta*spsi - sphi*cpsi,	-cphi*stheta*cpsi + sphi*spsi,	0,
				  sphi*ctheta,	-sphi*stheta*spsi + cphi*cpsi,	-sphi*stheta*cpsi - cphi*spsi,	0,
				  stheta,		ctheta*spsi,					ctheta*cpsi,					0};
	// build robot
	double rot[2] = {robot->angle1, robot->angle2};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// set trackwidth
	const double *pos1, *pos2;
	pos1 = dBodyGetPosition(_body[WHEEL1]);
	pos2 = dBodyGetPosition(_body[WHEEL2]);
	_trackwidth = sqrt(pow(pos2[0] - pos1[0], 2) + pow(pos2[1] - pos1[1], 2));

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int CNXT::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) { _body[i] = dBodyCreate(_world); }
	_geom[BODY] = new dGeomID[1];
	_geom[WHEEL1] = new dGeomID[1];
	_geom[WHEEL2] = new dGeomID[1];

	// adjust input height by body height
	if (fabs(z) < (_body_radius - EPSILON)) {
		x += R[2]*_body_height/2;
		y += R[6]*_body_height/2;
		z += R[10]*_body_height/2;
	}

    // input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = DEG2RAD(rot[i]);
	}

	// offset values for each body part[0-2] and joint[3-5] from center
	double f1[6] = {-_body_width/2 - _wheel_depth/2, 0, 0, -_body_width/2, 0, 0};
	double f2[6] = {_body_width/2 + _wheel_depth/2, 0, 0, _body_width/2, 0, 0};

	// build robot bodies
	this->build_body(x, y, z, R, 0);
	this->build_wheel(WHEEL1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R, 0);
	this->build_wheel(WHEEL2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R, 0);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = 0;
	_center[2] = 0;

    // joint for body to wheel 1
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY], _body[WHEEL1]);
	dJointSetHingeAnchor(_joint[0], R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x,
									R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
									R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[WHEEL1], R[0], R[4], R[8]);

    // joint for body to wheel2
	_joint[1] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[1], _body[BODY], _body[WHEEL2]);
	dJointSetHingeAnchor(_joint[1], R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
									R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
									R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
	dJointSetHingeAxis(_joint[1], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[WHEEL2], R[0], R[4], R[8]);

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2;
    dRFromAxisAndAngle(R_f, -1, 0, 0, _motor[JOINT1].theta);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 1, 0, 0, _motor[JOINT2].theta);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 ) {
		this->build_wheel(WHEEL1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R_f1, 0);
		this->build_wheel(WHEEL2, R[0]*f2[0] + x, R[4]*f2[0] + y, R[8]*f2[0] + z, R_f2, 0);
	}

    // motor for body to wheel 1
    _motor[JOINT1].id = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[JOINT1].id, _body[BODY], _body[WHEEL1]);
    dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
    dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
    dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT1].id);

	// motor for body to wheel 2
	_motor[JOINT2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT2].id, _body[BODY], _body[WHEEL2]);
	dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, R[0], R[4], R[8]);
	dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT2].id);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

double CNXT::getAngle(int id) {
	_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;
    return _motor[id].theta;
}

int CNXT::initParams(int disabled, int type) {
	_dof = 2;

	// create arrays for linkbots
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[_dof];
	_geom = new dGeomID * [NUM_PARTS];
	_joint = new dJointID[_dof];
	_motor.resize(_dof);

	// fill with default data
	for (int i = 0; i < _dof; i++) {
		_motor[i].accel.init = 0;
		_motor[i].accel.run = 0;
		_motor[i].accel.period = 0;
		_motor[i].accel.start = 0;
		_motor[i].alpha = 0;
		_motor[i].encoder = DEG2RAD(0.25);
		_motor[i].goal = 0;
		_motor[i].mode = SEEK;
		_motor[i].offset = 0;
		_motor[i].omega = 0.7854;			//  45 deg/sec
		_motor[i].omega_max = 4.1888;		// 240 deg/sec
		_motor[i].record = false;
		_motor[i].record_active = false;
		_motor[i].record_angle = new double * [_dof];
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
	_g_shift_data = 0;
	_g_shift_data_en = 0;
	_id = -1;
	_motion = false;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 1;
	_shift_data = 0;
	_speed = 2;
	_trace = 1;
	_type = type;

	// success
	return 0;
}

int CNXT::initDims(void) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_radius = _body_height/2;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;

	// success
	return 0;
}

void CNXT::simPreCollisionThread(void) {
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
		dJointGetHingeAxis(_joint[i], axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
		// engage motor depending upon motor mode
		double t = 0, angle = 0, h = 0, dt = 0;
		//double step = g_sim->getStep();
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
					//_motor[i].accel.start = g_sim->getClock();
					_motor[i].accel.start = _sim->getClock();
					_motor[i].accel.run = 1;
					break;
				}

				// calculate new angle
				h = _motor[i].goal - _motor[i].accel.init;
				//t = g_sim->getClock();
				t = _sim->getClock();
				dt = (t - _motor[i].accel.start)/_motor[i].accel.period;
				if (_motor[i].mode == ACCEL_CYCLOIDAL)
					angle = h*(dt - sin(2*M_PI*dt)/2/M_PI) + _motor[i].accel.init;
				else if (_motor[i].mode == ACCEL_HARMONIC)
					angle = h*(1 - cos(M_PI*dt))/2 + _motor[i].accel.init;

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
				if ((_motor[i].goal - 6*_motor[i].encoder - _motor[i].theta) > EPSILON) {
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
				else if ((_motor[i].goal - 3*_motor[i].encoder - _motor[i].theta) > EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].goal - _motor[i].encoder - _motor[i].theta) > EPSILON) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega)/4);
				}
				else if ((_motor[i].theta - _motor[i].goal - 6*_motor[i].encoder) > EPSILON) {
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
				else if ((_motor[i].theta - _motor[i].goal - 3*_motor[i].encoder) > EPSILON) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega)/2);
				}
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder) > EPSILON) {
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

void CNXT::simPostCollisionThread(void) {
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
int CNXT::build_body(double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m, m1, m2;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[BODY], x, y, z);
	dBodySetRotation(_body[BODY], R);
	dBodySetFiniteRotationMode(_body[BODY], 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry 1 - box
	_geom[BODY][0] = dCreateBox(_space, _body_width, _body_length, _body_height);
	dGeomSetBody(_geom[BODY][0], _body[BODY]);
	dGeomSetOffsetPosition(_geom[BODY][0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[BODY], &m);

	// success
	return 0;
}

int CNXT::build_wheel(int id, double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of wheel
	dMassSetCylinder(&m, 270, 1, 2*_wheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[id], x, y, z);
	dBodySetRotation(_body[id], R);
	dBodySetFiniteRotationMode(_body[id], 1);

	// rotation matrix
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry
	_geom[id][0] = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(_geom[id][0], _body[id]);
	dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[id][0], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// success
	return 0;
}

