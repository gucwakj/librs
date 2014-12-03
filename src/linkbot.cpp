#include "linkbot.hpp"

CLinkbotT::CLinkbotT(int disabled, int type) : Robot(JOINT1, JOINT3) {
	// initialize parameters
	this->initParams(disabled, type);

	// initialize dimensions
	this->initDims();
}

CLinkbotT::~CLinkbotT(void) {
	// remove robot from simulation
	//if ( g_sim != NULL && !(g_sim->deleteRobot(_pos)) )
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

int CLinkbotT::accelJointAngleNB(robotJointId_t id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int CLinkbotT::accelJointCycloidalNB(robotJointId_t id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	//_motor[id].timeout = t/g_sim->getStep();
	_motor[id].timeout = t/_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_CYCLOIDAL;
	_motor[id].goal = DEG2RAD(angle);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::accelJointHarmonicNB(robotJointId_t id, double angle, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( angle > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	//_motor[id].timeout = t/g_sim->getStep();
	_motor[id].timeout = t/_sim->getStep();

	// set acceleration parameters
	_motor[id].mode = ACCEL_HARMONIC;
	_motor[id].goal = DEG2RAD(angle) - DEG2RAD(2);
	_motor[id].accel.init = _motor[id].theta;
	_motor[id].accel.period = t;
	_motor[id].accel.run = 0;
	_motor[id].accel.start = 0;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::accelJointSmoothNB(robotJointId_t id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = DEG2RAD(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CLinkbotT::accelJointTimeNB(robotJointId_t id, double a, double t) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set initial omega
	if (_motor[id].state != POSITIVE || _motor[id].state != NEGATIVE) {
		if ( a > EPSILON )
			_motor[id].omega = 0.01;
		else
			_motor[id].omega = -0.01;
	}

	// set timeout
	//double step = g_sim->getStep();
	double step = _sim->getStep();
	if (t == 0)
		_motor[id].timeout = fabs((_motor[id].omega_max-fabs(_motor[id].omega))/DEG2RAD(a)/step);
	else
		_motor[id].timeout = fabs(t/step);

	// set acceleration parameters
	_motor[id].alpha = DEG2RAD(a);
	_motor[id].mode = ACCEL_CONSTANT;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
    dBodyEnable(_body[BODY]);
	MUTEX_UNLOCK(&_theta_mutex);

	// unsuccessful
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int CLinkbotT::accelJointToMaxSpeedNB(robotJointId_t id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int CLinkbotT::accelJointToVelocityNB(robotJointId_t id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int CLinkbotT::closeGripper(void) {
	double gripperAngleOld = 0;
	double gripperAngleNew = 0;
	int retval = getJointAngleInstant(JOINT1, gripperAngleNew);
	while ( fabs(gripperAngleNew - gripperAngleOld) > 0.1 ) {
		gripperAngleOld = gripperAngleNew;
		retval = retval || getJointAngleInstant(JOINT1, gripperAngleNew);
		retval = retval || moveNB(8, 0, 8);
		delaySeconds(1);
		retval = retval || getJointAngleInstant(JOINT1, gripperAngleNew);
	}
	retval = retval || moveNB(8, 0, 8);
	delaySeconds(1);
	retval = retval || holdJoints();
	return retval;
}

int CLinkbotT::closeGripperNB(void) {
	// create thread
	THREAD_T moving;

	// store args
	LinkbotMove *move = new LinkbotMove;
	move->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&moving, closeGripperNBThread, (void *)move);

	// success
	return 0;
}

int CLinkbotT::driveAccelCycloidalNB(double radius, double d, double t) {
	this->accelJointCycloidalNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointCycloidalNB(JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelDistanceNB(double radius, double a, double d) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CLinkbotT::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointHarmonicNB(JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(JOINT1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(JOINT3, a0, af, vmax, d/radius);

	// success
	return 0;
}

int CLinkbotT::driveAccelTimeNB(double radius, double a, double t) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), t);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelToMaxSpeedNB(double radius, double a) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), 0);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), 0);

	// success
	return 0;
}

int CLinkbotT::driveAccelToVelocityNB(double radius, double a, double v) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(JOINT1,  RAD2DEG(a/radius), v/a);
	this->accelJointTimeNB(JOINT3, -RAD2DEG(a/radius), v/a);

	// success
	return 0;
}

int CLinkbotT::driveForeverNB(void) {
	// negate speed to act as a car
	_motor[JOINT3].omega = -_motor[JOINT3].omega;

	// set joint movements
	this->moveJointForeverNB(JOINT1);
	this->moveJointForeverNB(JOINT3);

	// success
	return 0;
}

int CLinkbotT::driveForwardNB(double angle) {
	this->moveJointNB(JOINT1, angle);
	this->moveJointNB(JOINT3, -angle);

	// success
	return 0;
}

int CLinkbotT::drivexyTo(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// if movement is too small, just call it good
	if (fabs(x-x0) < 0.1 && fabs(y-y0) < 0.1) {
		return 1;
	}

	// get current rotation
	double r0 = this->getRotation(BODY, 2);

	// compute rotation matrix for body frame
	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// get angle to turn in body coordinates (transform of R)
	double angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

	// get speed of robot
	double *speed = new double[_dof]();
	this->getJointSpeeds(speed[0], speed[1], speed[2]);

	if (fabs(speed[0]) > 120) {
		this->setJointSpeeds(45, 45, 45);
	}

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.005) {
		// turn in shortest path
		if (angle > 0.005)
			this->turnRight(RAD2DEG(angle), radius, trackwidth);
		else if (angle < -0.005)
			this->turnLeft(RAD2DEG(-angle), radius, trackwidth);

		// calculate new rotation from error
		this->getxy(x0, y0);
		r0 = this->getRotation(BODY, 2);
		dRSetIdentity(R);
		dRFromAxisAndAngle(R, 0, 0, 1, r0);
		angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

		// move slowly
		this->setJointSpeeds(45, 45, 45);
	}

	// reset to original speed after turning
	this->setJointSpeeds(speed[0], speed[1], speed[2]);

	// move along length of line
	this->getxy(x0, y0);
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// clean up
	delete speed;

	// success
	return 0;
}

int CLinkbotT::drivexyToSmooth(double x1, double y1, double x2, double y2, double x3, double y3, double radius, double trackwidth) {
	// get midpoints
	double p[2] = {(x1 + x2)/2, (y1 + y2)/2};
	double q[2] = {(x2 + x3)/2, (y2 + y3)/2};

	// calculate equations of bisecting lines
	double m1 = -1/((y1 - y2)/(x1 - x2));
	double m2 = -1/((y2 - y3)/(x2 - x3));
	double b1 = -m1*p[0] + p[1];
	double b2 = -m2*q[0] + q[1];

	// circle parameters that passes through these points
	double c[2] = {(b2 - b1)/(m1 - m2), m1*(b2 - b1)/(m1 - m2) + b1};
	double rho = sqrt((c[0] - x2)*(c[0] - x2) + (c[1] - y2)*(c[1] - y2));
	double theta = 2*fabs(atan((m1 - m2)/(1 + m1*m2)));

	// distance to travel for each wheel
	trackwidth = this->convert(_trackwidth, 0);
	double s1 = theta*(rho + trackwidth/2);
	double s2 = theta*(rho - trackwidth/2);

	// move joints the proper amount
	this->setJointSpeed(JOINT1, RAD2DEG(s1/theta/rho/radius*_speed));
	this->setJointSpeed(JOINT3, RAD2DEG(s2/theta/rho/radius*_speed));
	this->moveJointNB(JOINT1, RAD2DEG(s1/radius));
	this->moveJointNB(JOINT3, -RAD2DEG(s2/radius));
	this->delay(theta*rho/_speed*1000);

	// success
	return 0;
}

int CLinkbotT::getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);
	this->getJointAngle(JOINT3, angle3, numReadings);

	// success
	return 0;
}

int CLinkbotT::getJointAnglesInstant(double &angle1, double &angle2, double &angle3) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);
	this->getJointAngleInstant(JOINT3, angle3);

	// success
	return 0;
}

int CLinkbotT::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);
	speed3 = RAD2DEG(_motor[JOINT3].omega);

	// success
	return 0;
}

int CLinkbotT::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;
	ratio3 = _motor[JOINT3].omega/_motor[JOINT3].omega_max;

	// success
	return 0;
}

int CLinkbotT::move(double angle1, double angle2, double angle3) {
	this->moveNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveNB(double angle1, double angle2, double angle3) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CLinkbotT::moveTo(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveToNB(double angle1, double angle2, double angle3) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CLinkbotT::moveToByTrackPos(double angle1, double angle2, double angle3) {
	this->moveToByTrackPosNB(angle1, angle2, angle3);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::moveToByTrackPosNB(double angle1, double angle2, double angle3) {
	this->moveToNB(angle1, angle2, angle3);

	// success
	return 0;
}

int CLinkbotT::openGripper(double angle) {
	this->openGripperNB(angle);
	this->moveWait();

	// success
	return 0;
}

int CLinkbotT::openGripperNB(double angle) {
	if (_type == LINKBOTL)
		this->moveJointToNB(JOINT1, -angle);
	else
		this->moveToNB(-angle/2, 0, -angle/2);

	// success
	return 0;
}

int CLinkbotT::recordAngles(double *time, double *angle1, double *angle2, double *angle3, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CLinkbotT::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbotT::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;
	angles[JOINT3] = distance3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbotT::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);

	// success
	return 0;
}

int CLinkbotT::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);

	// success
	return 0;
}

int CLinkbotT::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = this->convert(_trackwidth, 0);

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(-angle, 0, -angle);

	// success
	return 0;
}

int CLinkbotT::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = this->convert(_trackwidth, 0);

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move
	this->moveNB(angle, 0, angle);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CLinkbotT::addConnector(int type, int face, double size) {
	// create new connector
	_conn.push_back(new Connector());
	_conn.back()->d_side = -1;
	_conn.back()->d_type = -1;
	_conn.back()->face = face;
	_conn.back()->type = type;

	// build connector
	switch (type) {
		case BIGWHEEL:
			this->build_bigwheel(_conn.back(), face);
			break;
		case BRIDGE:
			this->build_bridge(_conn.back(), face);
			break;
		case CASTER:
			this->build_caster(_conn.back(), face, static_cast<int>(size));
			break;
		case CUBE:
			this->build_cube(_conn.back(), face);
			break;
		case FACEPLATE:
			this->build_faceplate(_conn.back(), face);
			break;
		case GRIPPER:
			this->build_gripper(_conn.back(), 1);
			break;
		case OMNIDRIVE:
			this->build_omnidrive(_conn.back(), face);
			break;
		case SIMPLE:
			this->build_simple(_conn.back(), face);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(_conn.back(), face);
			break;
		case TINYWHEEL:
			this->build_tinywheel(_conn.back(), face);
			break;
		case WHEEL:
			this->build_wheel(_conn.back(), face, size);
			break;
	}

	if (type == GRIPPER) {
		_conn.push_back(new Connector());
		_conn.back()->d_side = -1;
		_conn.back()->d_type = -1;
		_conn.back()->face = face;
		_conn.back()->type = type;
		this->build_gripper(_conn.back(), 3);
	}

	// success
	return 0;
}

int CLinkbotT::build(XMLRobot *robot, int really) {
	// check for wheels
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->conn == BIGWHEEL) {
			robot->z += (_bigwheel_radius - _body_height/2);
			_radius = _bigwheel_radius;
			break;
		}
		else if (robot->conn[i]->conn == SMALLWHEEL) {
			robot->z += (_smallwheel_radius - _body_height/2);
			_radius = _smallwheel_radius;
			break;
		}
		else if (robot->conn[i]->conn == TINYWHEEL) {
			robot->z += (_tinywheel_radius - _body_height/2);
			_radius = _tinywheel_radius;
			break;
		}
		else if (robot->conn[i]->conn == WHEEL) {
			robot->z += (robot->conn[i]->size - _body_height/2);
			_radius = robot->conn[i]->size;
			if (fabs(robot->z) > (_body_radius-EPSILON)) { robot->z += _body_height/2; }
			break;
		}
	}
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->conn == CASTER && !static_cast<int>(robot->conn[i]->size))
			robot->psi += RAD2DEG(atan2(_radius - _smallwheel_radius, 0.08575));
	}

	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta,	-cphi*stheta*spsi - sphi*cpsi,	-cphi*stheta*cpsi + sphi*spsi,	0,
				  sphi*ctheta,	-sphi*stheta*spsi + cphi*cpsi,	-sphi*stheta*cpsi - cphi*spsi,	0,
				  stheta,		ctheta*spsi,					ctheta*cpsi,					0};
	dRtoQ(R, robot->q);

	// adjust input height by body height
	if (fabs(robot->z) < (_body_radius - EPSILON)) {
		robot->x += R[2]*_body_height/2;
		robot->y += R[6]*_body_height/2;
		robot->z += R[10]*_body_height/2;
	}

	// build robot
	double rot[3] = {robot->angle1, robot->angle2, robot->angle3};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// add connectors
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->robot == _id) {
			if (robot->conn[i]->conn == -1)
				this->addConnector(robot->conn[i]->type, robot->conn[i]->face1, robot->conn[i]->size);
			else
				this->add_connector_daisy(robot->conn[i]->conn, robot->conn[i]->face1, robot->conn[i]->size, robot->conn[i]->side, robot->conn[i]->type);
		}
	}

	// set trackwidth
	double wheel[4] = {0};
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
	_trackwidth = sqrt(pow(wheel[0] - wheel[2], 2) + pow(wheel[1] - wheel[3], 2));

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int CLinkbotT::build(XMLRobot *robot, dMatrix3 R, double *m, dBodyID base, XMLConn *conn) {
	// initialize new variables
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, R5, R6;

	// generate parameters for connector
	this->getConnectorParams(conn->type, conn->side, R, m);

	// rotate about connection joint
	dRFromAxisAndAngle(R1, R[0], R[4], R[8], robot->psi);
	dMultiply0(R2, R1, R, 3, 3, 3);

	// rotate body for connection face
	switch (conn->face2) {
		case 1:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], 0);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(robot->angle1));
			break;
		case 2:
			offset[0] = _face_depth + _body_length;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], -M_PI/2);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -DEG2RAD(robot->angle2));
			break;
		case 3:
			offset[0] = _body_width/2 + _face_depth;
			dRFromAxisAndAngle(R3, R2[2], R2[6], R2[10], M_PI);
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[0], R4[4], R4[8], DEG2RAD(robot->angle3));
			break;
	}
	m[0] += R[0]*offset[0];
	m[1] += R[4]*offset[0];
	m[2] += R[8]*offset[0];
	dMultiply0(R6, R5, R4, 3, 3, 3);

    // build new module
	double rot[3] = {robot->angle1, robot->angle2, robot->angle3};
	this->buildIndividual(m[0], m[1], m[2], R6, rot);

    // add fixed joint to attach two modules
	this->fixBodyToConnector(base, conn->face2);

	// add connectors
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->robot == _id) {
			if (robot->conn[i]->conn == -1)
				this->addConnector(robot->conn[i]->type, robot->conn[i]->face1, robot->conn[i]->size);
			else
				this->add_connector_daisy(robot->conn[i]->conn, robot->conn[i]->face1, robot->conn[i]->size, robot->conn[i]->side, robot->conn[i]->type);
		}
		else if (robot->conn[i]->face2 != conn->face2) {
			this->fixConnectorToBody(robot->conn[i]->face2, base);
		}
	}

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int CLinkbotT::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) { _body[i] = dBodyCreate(_world); }
	_geom[BODY] = new dGeomID[2];
	_geom[FACE1] = new dGeomID[1];
	_geom[FACE2] = new dGeomID[1];
	_geom[FACE3] = new dGeomID[1];

    // input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = DEG2RAD(rot[i]);
	}

	// offset values for each body part[0-2] and joint[3-5] from center
	double f1[6] = {-_body_width/2 - _face_depth/2, 0, 0, -_body_width/2, 0, 0};
	double f2[6] = {0, -_body_length - _face_depth/2, 0, 0, -_body_length, 0};
	double f3[6] = {_body_width/2 + _face_depth/2, 0, 0, _body_width/2, 0, 0};

	// build robot bodies
	this->build_body(x, y, z, R, 0);
	this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R, 0);
	this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R, 0);
	this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R, 0);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = 0.012462;
	_center[2] = 0;

    // joint for body to face 1
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY], _body[FACE1]);
	dJointSetHingeAnchor(_joint[0], R[0]*f1[3] + R[1]*f1[4] + R[2]*f1[5] + x,
									R[4]*f1[3] + R[5]*f1[4] + R[6]*f1[5] + y,
									R[8]*f1[3] + R[9]*f1[4] + R[10]*f1[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);
	dBodySetFiniteRotationAxis(_body[FACE1], R[0], R[4], R[8]);

    // joint for body to face 2
	if (_disabled == 1) {
		dJointID joint = dJointCreateFixed(_world, 0);
		dJointAttach(joint, _body[BODY], _body[FACE2]);
		dJointSetFixed(joint);
	}
	else {
		_joint[1] = dJointCreateHinge(_world, 0);
		dJointAttach(_joint[1], _body[BODY], _body[FACE2]);
		dJointSetHingeAnchor(_joint[1], R[0]*f2[3] + R[1]*f2[4] + R[2]*f2[5] + x,
										R[4]*f2[3] + R[5]*f2[4] + R[6]*f2[5] + y,
										R[8]*f2[3] + R[9]*f2[4] + R[10]*f2[5] + z);
		dJointSetHingeAxis(_joint[1], R[1], R[5], R[9]);
		dBodySetFiniteRotationAxis(_body[FACE2], R[1], R[5], R[9]);
	}

    // joint for body to face 3
	if (_disabled == 2) {
		dJointID joint = dJointCreateFixed(_world, 0);
		dJointAttach(joint, _body[BODY], _body[FACE3]);
		dJointSetFixed(joint);
	}
	else {
		_joint[2] = dJointCreateHinge(_world, 0);
		dJointAttach(_joint[2], _body[BODY], _body[FACE3]);
		dJointSetHingeAnchor(_joint[2], R[0]*f3[3] + R[1]*f3[4] + R[2]*f3[5] + x,
										R[4]*f3[3] + R[5]*f3[4] + R[6]*f3[5] + y,
										R[8]*f3[3] + R[9]*f3[4] + R[10]*f3[5] + z);
		dJointSetHingeAxis(_joint[2], -R[0], -R[4], -R[8]);
		dBodySetFiniteRotationAxis(_body[FACE3], -R[0], -R[4], -R[8]);
	}

    // create rotation matrices for each body part
    dMatrix3 R_f, R_f1, R_f2, R_f3;
    dRFromAxisAndAngle(R_f, -1, 0, 0, _motor[JOINT1].theta);
    dMultiply0(R_f1, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 0, -1, 0, _motor[JOINT2].theta);
    dMultiply0(R_f2, R, R_f, 3, 3, 3);
	dRSetIdentity(R_f);
    dRFromAxisAndAngle(R_f, 1, 0, 0, _motor[JOINT3].theta);
    dMultiply0(R_f3, R, R_f, 3, 3, 3);

	// if bodies are rotated, then redraw
	if ( _motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 || _motor[JOINT3].theta != 0 ) {
		this->build_face(FACE1, R[0]*f1[0] + x, R[4]*f1[0] + y, R[8]*f1[0] + z, R_f1, 0);
		this->build_face(FACE2, R[1]*f2[1] + x, R[5]*f2[1] + y, R[9]*f2[1] + z, R_f2, 0);
		this->build_face(FACE3, R[0]*f3[0] + x, R[4]*f3[0] + y, R[8]*f3[0] + z, R_f3, 0);
	}

    // motor for body to face 1
    _motor[JOINT1].id = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[JOINT1].id, _body[BODY], _body[FACE1]);
    dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
    dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
    dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R[0], R[4], R[8]);
    dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
    dJointSetAMotorParam(_motor[JOINT1].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT1].id);

    // motor for body to face 2
    _motor[JOINT2].id = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[JOINT2].id, _body[BODY], _body[FACE2]);
    dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
    dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
    dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, R[1], R[5], R[9]);
    dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
    dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
    dJointSetAMotorParam(_motor[JOINT2].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT2].id);

    // motor for body to face 3
    _motor[JOINT3].id = dJointCreateAMotor(_world, 0);
    dJointAttach(_motor[JOINT3].id, _body[BODY], _body[FACE3]);
    dJointSetAMotorMode(_motor[JOINT3].id, dAMotorUser);
    dJointSetAMotorNumAxes(_motor[JOINT3].id, 1);
    dJointSetAMotorAxis(_motor[JOINT3].id, 0, 1, -R[0], -R[4], -R[8]);
    dJointSetAMotorAngle(_motor[JOINT3].id, 0, 0);
    dJointSetAMotorParam(_motor[JOINT3].id, dParamFMax, _motor[JOINT3].tau_max);
    dJointSetAMotorParam(_motor[JOINT3].id, dParamFudgeFactor, 0.3);
	dJointDisable(_motor[JOINT3].id);

    // set damping on all bodies to 0.1
    for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CLinkbotT::fixBodyToConnector(dBodyID cBody, int face) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CLinkbotT::fixConnectorToBody(int face, dBodyID cBody, int conn) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// connector or body part
	dBodyID body;
	if (conn != -1)
		body = this->getConnectorBodyID(face);
	else
		body = this->getBodyID(face);

	// attach to correct body
	dJointAttach(joint, body, cBody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

double CLinkbotT::getAngle(int id) {
	if (id == _disabled)
		_motor[id].theta = 0;
	else
		_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;

    return _motor[id].theta;
}

int CLinkbotT::getConnectorParams(int type, int side, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};

	switch (type) {
		case BRIDGE:
			offset[1] = -_bridge_length + 2*_face_radius;
			dRFromAxisAndAngle(R1, R[1], R[5], R[9], M_PI);
			break;
		case CUBE:
			if (side == 2) {
				offset[0] = _cubic_length/2;
				offset[1] = _cubic_length/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI/2);
			}
			else if (side == 3) {
				offset[0] = _cubic_length;
				dRSetIdentity(R1);
			}
			else if (side == 4) {
				offset[0] = _cubic_length/2;
				offset[1] = -_cubic_length/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], -M_PI/2);
			}
			else if (side == 5) {
				offset[0] = _cubic_length/2;
				offset[2] = _cubic_length/2;
				dRFromAxisAndAngle(R2, R[1], R[5], R[9], -M_PI/2);
				dMultiply0(R3, R2, R, 3, 3, 3);
				dRFromAxisAndAngle(R4, R3[0], R3[4], R3[8], -M_PI/2);
				dMultiply0(R1, R4, R2, 3, 3, 3);
			}
			break;
		case OMNIDRIVE:
			if (side == 2) {
				offset[2] = -_omni_length + 2*_face_radius;
			}
			else if (side == 3) {
				offset[1] = +_omni_length - 2*_face_radius;
			}
			else if (side == 4) {
				offset[1] = _omni_length - 2*_face_radius;
				offset[2] = -_omni_length + 2*_face_radius;
			}
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI);
			break;
		case SIMPLE:
			offset[0] = _conn_depth;
			dRSetIdentity(R1);
			break;
	}

	// set output parameters
	p[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	p[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	p[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];
	dMultiply0(R, R1, Rtmp, 3, 3, 3);

	// success
	return 0;
}

int CLinkbotT::getFaceParams(int face, dMatrix3 R, double *p) {
	double offset[3] = {0};
	const double *pos = dBodyGetPosition(_body[face]);
	const double *R1 = dBodyGetRotation(_body[face]);
	dMatrix3 R2;

	// get offset and rotation of face connection
	switch (face) {
		case 1:
			offset[0] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], M_PI);
			break;
		case 2:
			offset[1] = -_face_depth/2;
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], -M_PI/2);
			break;
		case 3:
			offset[0] = _face_depth/2;
			dRSetIdentity(R2);
			break;
	}

	// generate new position
	p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
	p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
	p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
	// generate new rotation matrix
	dMultiply0(R, R2, R1, 3, 3, 3);

	// success
	return 0;
}

int CLinkbotT::initParams(int disabled, int type) {
	_dof = 3;

	// create arrays for linkbots
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[(disabled == -1) ? 3 : 2];
	_geom = new dGeomID * [NUM_PARTS];
	_joint = new dJointID[_dof];
	_motor.resize(_dof);
	_neighbor.resize(_dof);

	// fill with default data
	for (int i = 0, j = 0; i < _dof; i++) {
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
		if (i != disabled) { _enabled[j++] = i; }
	}
	_connected = 0;
	_disabled = disabled;
	_distOffset = 0;
	_g_shift_data = 0;
	_g_shift_data_en = 0;
	_id = -1;
	_motion = false;
	_rgb[0] = 0;
	_rgb[1] = 0;
	_rgb[2] = 1;
	_shift_data = 0;
	_sim = NULL;
	_speed = 2;
	_trace = 1;
	_type = type;

	// success
	return 0;
}

int CLinkbotT::initDims(void) {
	_body_length = 0.03935;
	_body_width = 0.07835;
	_body_height = 0.07250;
	_body_radius = 0.03625;
	_face_depth = 0.00200;
	_face_radius = 0.03060;
	_conn_depth = 0.00570;
	_conn_height = 0.03715;
	_bigwheel_radius = 0.05080;
	_bridge_length = 0.14025;
	_cubic_length = 0.07115;
	_omni_length = 0.17360;
	_radius = _body_height/2;
	_smallwheel_radius = 0.04445;
	_tinywheel_radius = 0.04128;
	_wheel_depth = 0.00140;
	_wheel_radius = 0.04445;
	_offset.push_back(Vec3(0, 0, 0));									// body
	_offset.push_back(Vec3(-_body_width/2 - _face_depth/2, 0, 0));		// face1
	_offset.push_back(Vec3(0, -_body_length - _face_depth/2, 0));		// face2
	_offset.push_back(Vec3(_body_width/2 + _face_depth/2, 0, 0));		// face3


	// success
	return 0;
}

void CLinkbotT::simPreCollisionThread(void) {
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
	for (int j = 0; j < ((_disabled == -1) ? 3 : 2); j++) {
		int i = _enabled[j];
		// store current angle
		_motor[i].theta = getAngle(i);
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_joint[i], axis);
		dBodySetFiniteRotationAxis(_body[i+1], axis[0], axis[1], axis[2]);
		for (int k = 0; k < _conn.size(); k++) {
			if (_conn[k]->face == i+1)
				dBodySetFiniteRotationAxis(_conn[k]->body, axis[0], axis[1], axis[2]);
		}
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
				else if ((_motor[i].goal - _motor[i].encoder/2 - _motor[i].theta) > EPSILON) {
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
				else if ((_motor[i].theta - _motor[i].goal - _motor[i].encoder/2) > EPSILON) {
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

void CLinkbotT::simPostCollisionThread(void) {
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
int CLinkbotT::add_connector_daisy(int conn, int face, double size, int side, int type) {
	// create new connector
	_conn.push_back(new Connector());
	_conn.back()->d_side = side;
	_conn.back()->d_type = type;
	_conn.back()->face = face;
	_conn.back()->type = conn;

	// build connector
	switch (conn) {
		case BIGWHEEL:
			this->build_bigwheel(_conn.back(), face, side, type);
			break;
		case BRIDGE:
			this->build_bridge(_conn.back(), face, side, type);
			break;
		case CASTER:
			_conn.back()->d_side = -10*size;
			this->build_caster(_conn.back(), face, static_cast<int>(size), side, type);
			break;
		case CUBE:
			this->build_cube(_conn.back(), face, side, type);
			break;
		case FACEPLATE:
			this->build_faceplate(_conn.back(), face, side, type);
			break;
		case OMNIDRIVE:
			this->build_omnidrive(_conn.back(), face, side, type);
			break;
		case SIMPLE:
			this->build_simple(_conn.back(), face, side, type);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(_conn.back(), face, side, type);
			break;
		case TINYWHEEL:
			this->build_tinywheel(_conn.back(), face, side, type);
			break;
		case WHEEL:
			this->build_wheel(_conn.back(), face, size, side, type);
			break;
	}

	// success
	return 0;
}

int CLinkbotT::build_bigwheel(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _wheel_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_bigwheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _bigwheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_body(double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m, m1, m2;
	dMatrix3 R1, R2, R3;

	// set mass of body
	dMassSetBox(&m, 1000, _body_width, _body_length, _body_height);
	dMassTranslate(&m, 0, -_body_length/2, 0);
	dMassSetCylinder(&m2, 400, 1, _body_radius, _body_width);
	dMassAdd(&m, &m2);

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
	dGeomSetOffsetPosition(_geom[BODY][0], -m.c[0], -_body_length/2 - m.c[1], -m.c[2]);

	// set geometry 2 - cylinder
	_geom[BODY][1] = dCreateCylinder(_space, _body_radius, _body_width);
	dGeomSetBody(_geom[BODY][1], _body[BODY]);
	dGeomSetOffsetPosition(_geom[BODY][1], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[BODY][1], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[BODY], &m);

	// success
	return 0;
}

int CLinkbotT::build_bridge(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _conn_depth/2;
	conn->o[1] = -_bridge_length/2 + _face_radius;
	conn->o[2] = 0;
	if (face == 3) conn->o[1] = _bridge_length/2 - _face_radius;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0] + R[1]*conn->o[1];
	p[1] += R[4]*conn->o[0] + R[5]*conn->o[1];
	p[2] += R[8]*conn->o[0] + R[9]*conn->o[1];

	// set mass of body
	dMassSetBox(&m, 270, _conn_depth, _bridge_length, _conn_height);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _conn_depth, _bridge_length, _conn_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_caster(Connector *conn, int face, int custom, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[4];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _conn_depth/4;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetBox(&m, 1000, 2*_conn_depth, 1.5*_face_radius, _body_height);
	dMassTranslate(&m, 8*_conn_depth, 0, -_body_height/2);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry 1 - box
	conn->geom[0] = dCreateBox(_space, _conn_depth, 1.5*_face_radius, _body_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// default 3ds caster
	if (!custom) {
		// set geometry 2 - horizontal support
		conn->geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(conn->geom[1], conn->body);
		dGeomSetOffsetPosition(conn->geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 3 - ball support
		conn->geom[2] = dCreateCylinder(_space, 0.011, 0.003);
		dGeomSetBody(conn->geom[2], conn->body);
		dGeomSetOffsetPosition(conn->geom[2], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 + 0.0001 - m.c[2]);

		// set geometry 4 - sphere
		conn->geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(conn->geom[3], conn->body);
		dGeomSetOffsetPosition(conn->geom[3], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 - 0.005 - m.c[2]);
	}
	// custom drawn one for mathematics
	else {
		// set geometry 2 - horizontal support
		conn->geom[1] = dCreateBox(_space, 0.02, 0.022, 0.0032);
		dGeomSetBody(conn->geom[1], conn->body);
		dGeomSetOffsetPosition(conn->geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 3 - ball support
		conn->geom[2] = dCreateCylinder(_space, 0.011, _radius -_face_radius - 0.006 + 0.0032);
		dGeomSetBody(conn->geom[2], conn->body);
		dGeomSetOffsetPosition(conn->geom[2], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 - (_radius -_face_radius - 0.006)/2 + 0.0016 - m.c[2]);

		// set geometry 4 - sphere
		conn->geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(conn->geom[3], conn->body);
		dGeomSetOffsetPosition(conn->geom[3], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 + _face_radius - _radius + 0.006 - m.c[2]);
	}

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_cube(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _cubic_length/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetBox(&m, 270, _cubic_length, _cubic_length, _cubic_length);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _cubic_length, _cubic_length, _cubic_length);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_face(int id, double x, double y, double z, dMatrix3 R, double theta) {
	// define parameters
	dMass m;
	dMatrix3 R1, R2, R3;

	// set mass of body
	if (id == 2)
		dMassSetCylinder(&m, 270, 2, 2*_face_radius, _face_depth);
	else
		dMassSetCylinder(&m, 270, 1, 2*_face_radius, _face_depth);

	// adjust x,y,z to position center of mass correctly
	x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(_body[id], x, y, z);
	dBodySetRotation(_body[id], R);
	dBodySetFiniteRotationMode(_body[id], 1);

	// rotation matrix
	if (id == 2)
	    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	else
	    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);		// SWITCHED X AND Y AXIS
	dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
	dMultiply0(R2, R1, R3, 3, 3, 3);

	// set geometry
	_geom[id][0] = dCreateCylinder(_space, _face_radius, _face_depth);
	dGeomSetBody(_geom[id][0], _body[id]);
	dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(_geom[id][0], R2);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CLinkbotT::build_faceplate(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _conn_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetBox(&m, 270, _conn_depth, _body_height, _body_height);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _conn_depth, _body_height, _body_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_gripper(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[3];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};
	int i = (face == 1) ? 1 : -1;

	// store body offset of connector
	conn->o[0] = _conn_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetBox(&m, 270, _conn_depth, 2*_face_radius, _conn_height);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry 1
	conn->geom[0] = dCreateBox(_space, _conn_depth, 4*_face_radius, _conn_height/2);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetPosition(conn->geom[0], 0, -i*_face_radius, 0);

	// set geometry 2
	conn->geom[1] = dCreateBox(_space, 0.062, 0.04, _conn_depth);
	dGeomSetBody(conn->geom[1], conn->body);
	dGeomSetOffsetPosition(conn->geom[1], _conn_depth/2 - 0.062/2 - m.c[0],
							-i*3*_face_radius + i*0.02 - m.c[1],
							i*_conn_height/4 - i*_conn_depth/2 - m.c[2]);

	// set geometry 3
	conn->geom[2] = dCreateBox(_space, 0.0344, 0.04, 0.007);
	dGeomSetBody(conn->geom[2], conn->body);
	dGeomSetOffsetPosition(conn->geom[2], _conn_depth/2 - 0.062 + 0.0344/2 - m.c[0],
							-i*3*_face_radius + i*0.02 - m.c[1],
							i*_conn_height/4 - i*_conn_depth/2 - i*0.007/2 - m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CLinkbotT::build_omnidrive(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _conn_depth/2;
	conn->o[1] = _omni_length/2 - _face_radius;
	conn->o[2] = -_omni_length/2 + _face_radius;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0] + R[1]*conn->o[1] + R[2]*conn->o[2];
	p[1] += R[4]*conn->o[0] + R[5]*conn->o[1] + R[6]*conn->o[2];
	p[2] += R[8]*conn->o[0] + R[9]*conn->o[1] + R[10]*conn->o[2];

	// set mass of body
	dMassSetBox(&m, 270, _omni_length, _omni_length, _conn_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _conn_depth, _omni_length, _omni_length);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_simple(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _conn_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetBox(&m, 270, _conn_depth, 2*_face_radius, _conn_height);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// set geometry
	conn->geom[0] = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_smallwheel(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _wheel_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_smallwheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _smallwheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_tinywheel(Connector *conn, int face, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _wheel_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_tinywheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _tinywheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

int CLinkbotT::build_wheel(Connector *conn, int face, double size, int side, int type) {
	// create body
	conn->body = dBodyCreate(_world);
	conn->geom = new dGeomID[1];

	// store wheel radius
	_wheel_radius = size;

	// define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0};

	// store body offset of connector
	conn->o[0] = _wheel_depth/2;
	conn->o[1] = 0;
	conn->o[2] = 0;

	// position center of connector
	this->getFaceParams(face, R, p);
	if (side != -1) this->getConnectorParams(type, side, R, p);
	p[0] += R[0]*conn->o[0];
	p[1] += R[4]*conn->o[0];
	p[2] += R[8]*conn->o[0];

	// set mass of body
	dMassSetCylinder(&m, 270, 1, 2*_wheel_radius, _wheel_depth);

	// adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
	p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
	p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

	// set body parameters
	dBodySetPosition(conn->body, p[0], p[1], p[2]);
	dBodySetRotation(conn->body, R);
	dBodySetFiniteRotationMode(conn->body, 1);

	// rotation matrix for curves
	dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

	// set geometry
	conn->geom[0] = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(conn->geom[0], conn->body);
	dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
	dGeomSetOffsetRotation(conn->geom[0], R1);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body, side);

	// success
	return 0;
}

void* CLinkbotT::closeGripperNBThread(void *arg) {
	// cast arg
	LinkbotMove *move = (LinkbotMove *)arg;

	// perform motion
	move->robot->closeGripper();

	// signal successful completion
	SIGNAL(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

