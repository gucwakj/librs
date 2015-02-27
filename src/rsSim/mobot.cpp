#include <rsSim/Mobot>

CMobot::CMobot(void) : Robot(JOINT1, JOINT4) {
	// initialize parameters
	this->initParams(0, MOBOT);

	// initialize dimensions
	this->initDims();
}

CMobot::~CMobot(void) {
	// remove robot from simulation
	if ( _sim != NULL && !(_sim->deleteRobot(_pos)) )
		delete _sim;

	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}

	// remove geoms
	if (_connected) {
		for (int i = NUM_PARTS - 1; i >= 0; i--) { delete [] _geom[i]; }
	}
}

int CMobot::getJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, int numReadings) {
	this->getJointAngle(JOINT1, angle1, numReadings);
	this->getJointAngle(JOINT2, angle2, numReadings);
	this->getJointAngle(JOINT3, angle3, numReadings);
	this->getJointAngle(JOINT4, angle4, numReadings);

	// success
	return 0;
}

int CMobot::getJointAnglesInstant(double &angle1, double &angle2, double &angle3, double &angle4) {
	this->getJointAngleInstant(JOINT1, angle1);
	this->getJointAngleInstant(JOINT2, angle2);
	this->getJointAngleInstant(JOINT3, angle3);
	this->getJointAngleInstant(JOINT4, angle4);

	// success
	return 0;
}

int CMobot::getJointSpeeds(double &speed1, double &speed2, double &speed3, double &speed4) {
	speed1 = RAD2DEG(_motor[JOINT1].omega);
	speed2 = RAD2DEG(_motor[JOINT2].omega);
	speed3 = RAD2DEG(_motor[JOINT3].omega);
	speed4 = RAD2DEG(_motor[JOINT4].omega);

	// success
	return 0;
}

int CMobot::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3, double &ratio4) {
	ratio1 = _motor[JOINT1].omega/_motor[JOINT1].omega_max;
	ratio2 = _motor[JOINT2].omega/_motor[JOINT2].omega_max;
	ratio3 = _motor[JOINT3].omega/_motor[JOINT3].omega_max;
	ratio4 = _motor[JOINT4].omega/_motor[JOINT4].omega_max;

	// success
	return 0;
}

int CMobot::motionArch(double angle) {
	this->moveJointToNB(JOINT2, -angle/2.0);
	this->moveJointToNB(JOINT3, angle/2.0);
	this->moveJointWait(JOINT2);
	this->moveJointWait(JOINT3);

	// success
	return 0;
}

void* CMobot::motionArchThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionArch(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionArchNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionArchThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionDistance(double distance, double radius) {
	this->motionRollForward(distance/radius);

	// success
	return 0;
}

int CMobot::motionDistanceNB(double distance, double radius) {
	this->motionRollForwardNB(distance/radius);

	// success
	return 0;
}

int CMobot::motionInchwormLeft(int num) {
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT2, -50);
		this->moveJointTo(JOINT3, 50);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormLeftThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionInchwormLeft(move->i);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionInchwormLeftNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionInchwormLeftThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionInchwormRight(int num) {
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT3, 50);
		this->moveJointTo(JOINT2, -50);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
	}

	// success
	return 0;
}

void* CMobot::motionInchwormRightThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionInchwormRight(move->i);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionInchwormRightNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionInchwormRightThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionRollBackward(double angle) {
	this->move(-angle, 0, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionRollBackwardThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionRollBackward(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionRollBackwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollBackwardThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionRollForward(double angle) {
	double motorPosition[2];
	this->getJointAngleInstant(JOINT1, motorPosition[0]);
	this->getJointAngleInstant(JOINT4, motorPosition[1]);
	this->moveJointToNB(JOINT1, motorPosition[0] + angle);
	this->moveJointToNB(JOINT4, motorPosition[1] + angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionRollForwardThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionRollForward(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionRollForwardNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionRollForwardThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionSkinny(double angle) {
	this->moveJointToNB(JOINT2, angle);
	this->moveJointToNB(JOINT3, angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionSkinnyThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionSkinny(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionSkinnyNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionSkinnyThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionStand(void) {
	this->resetToZero();
	this->moveJointTo(JOINT2, -85);
	this->moveJointTo(JOINT3, 70);
	this->moveWait();
	this->moveJointTo(JOINT1, 45);
	this->doze(1000);
	this->moveJointTo(JOINT2, 20);

	// success
	return 0;
}

void* CMobot::motionStandThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionStand();

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionStandNB(void) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionStandThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionTumbleLeft(int num) {
	this->resetToZero();
	this->doze(1000);

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT2, -85);
		this->moveJointTo(JOINT3, 75);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 80);
		this->moveJointTo(JOINT2, 45);
		this->moveJointTo(JOINT3, -85);
		this->moveJointTo(JOINT2, 75);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 75);
		if (i != (num-1)) {
			this->moveJointTo(JOINT3, 45);
		}
	}
	this->moveJointToNB(JOINT2, 0);
	this->moveJointToNB(JOINT3, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleLeftThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionTumbleLeft(move->i);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTumbleLeftNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTumbleLeftThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionTumbleRight(int num) {
	this->resetToZero();
	this->doze(1000);

	for (int i = 0; i < num; i++) {
		this->moveJointTo(JOINT3, 85);
		this->moveJointTo(JOINT2, -80);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, -80);
		this->moveJointTo(JOINT3, -45);
		this->moveJointTo(JOINT2, 85);
		this->moveJointTo(JOINT3, -80);
		this->moveJointTo(JOINT2, 0);
		this->moveJointTo(JOINT3, 0);
		this->moveJointTo(JOINT2, -80);
		if (i != (num-1)) {
			this->moveJointTo(JOINT2, -45);
		}
	}
	this->moveJointToNB(JOINT3, 0);
	this->moveJointToNB(JOINT2, 0);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTumbleRightThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionTumbleRight(move->i);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTumbleRightNB(int num) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->i = num;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTumbleRightThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionTurnLeft(double angle) {
	this->move(-angle, 0, 0, angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTurnLeftThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionTurnLeft(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTurnLeftNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnLeftThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionTurnRight(double angle) {
	this->move(angle, 0, 0, -angle);
	this->moveWait();

	// success
	return 0;
}

void* CMobot::motionTurnRightThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionTurnRight(move->d);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionTurnRightNB(double angle) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;
	move->d = angle;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionTurnRightThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionUnstand(void) {
	this->resetToZero();
	this->moveJointToNB(JOINT3, 45);
	this->moveJointToNB(JOINT2, -85);
	this->moveWait();
	this->resetToZero();

	// success
	return 0;
}

void* CMobot::motionUnstandThread(void *arg) {
	// cast arg
	MobotMove *move = (MobotMove *)arg;

	// perform motion
	move->robot->motionUnstand();

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// success
	return NULL;
}

int CMobot::motionUnstandNB(void) {
	// create thread
	THREAD_T motion;

	// store args
	MobotMove *move = new MobotMove;
	move->robot = this;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&motion, motionUnstandThread, (void *)move);

	// cleanup
	delete move;

	// success
	return 0;
}

int CMobot::motionWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int CMobot::move(double angle1, double angle2, double angle3, double angle4) {
	this->moveNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveNB(double angle1, double angle2, double angle3, double angle4) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;
	angles[JOINT4] = angle4;

	// call base class recording function
	int retval = Robot::moveNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CMobot::moveTo(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	// store angles
	double *angles = new double[_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;
	angles[JOINT4] = angle4;

	// call base class recording function
	int retval = Robot::moveToNB(angles);

	// clean up
	delete angles;

	// success
	return retval;
}

int CMobot::moveToByTrackPos(double angle1, double angle2, double angle3, double angle4) {
	this->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	this->moveWait();

	// success
	return 0;
}

int CMobot::moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4) {
	this->moveToNB(angle1, angle2, angle3, angle4);

	// success
	return 0;
}

int CMobot::recordAngles(double *time, double *angle1, double *angle2, double *angle3, double *angle4, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;
	angles[JOINT4] = angle4;

	// call base class recording function
	return Robot::recordAngles(time, angles, num, seconds, shiftData);
}

int CMobot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t &angle1, robotRecordData_t &angle2, robotRecordData_t &angle3, robotRecordData_t &angle4, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = angle1;
	angles[JOINT2] = angle2;
	angles[JOINT3] = angle3;
	angles[JOINT4] = angle4;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CMobot::recordDistancesBegin(robotRecordData_t &time, robotRecordData_t &distance1, robotRecordData_t &distance2, robotRecordData_t &distance3, robotRecordData_t &distance4, double radius, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// store angles
	double **angles = new double * [_dof];
	angles[JOINT1] = distance1;
	angles[JOINT2] = distance2;
	angles[JOINT3] = distance3;
	angles[JOINT4] = distance4;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CMobot::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	this->setJointSpeed(JOINT1, speed1);
	this->setJointSpeed(JOINT2, speed2);
	this->setJointSpeed(JOINT3, speed3);
	this->setJointSpeed(JOINT4, speed4);

	// success
	return 0;
}

int CMobot::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	this->setJointSpeedRatio(JOINT1, ratio1);
	this->setJointSpeedRatio(JOINT2, ratio2);
	this->setJointSpeedRatio(JOINT3, ratio3);
	this->setJointSpeedRatio(JOINT4, ratio4);

	// success
	return 0;
}

/**********************************************************
	inherited functions
 **********************************************************/
int CMobot::addConnector(int type, int face, double size) {
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
		case CASTER:
			this->build_caster(_conn.back(), face);
			break;
		case SIMPLE:
			this->build_simple(_conn.back(), face);
			break;
		case SMALLWHEEL:
			this->build_smallwheel(_conn.back(), face);
			break;
		case SQUARE:
			this->build_square(_conn.back(), face);
			break;
		case TANK:
			this->build_tank(_conn.back(), face);
			break;
		case WHEEL:
			this->build_wheel(_conn.back(), face, size);
			break;
	}

	// success
	return 0;
}

int CMobot::build(XMLRobot *robot, int really) {
	// create rotation matrix
	double	sphi = sin(DEG2RAD(robot->phi)),		cphi = cos(DEG2RAD(robot->phi)),
			stheta = sin(DEG2RAD(robot->theta)),	ctheta = cos(DEG2RAD(robot->theta)),
			spsi = sin(DEG2RAD(robot->psi)),		cpsi = cos(DEG2RAD(robot->psi));
	dMatrix3 R = {cphi*ctheta, -cphi*stheta*spsi - sphi*cpsi, -cphi*stheta*cpsi + sphi*spsi, 0,
				  sphi*ctheta, -sphi*stheta*spsi + cphi*cpsi, -sphi*stheta*cpsi - cphi*spsi, 0,
				  stheta, ctheta*spsi, ctheta*cpsi, 0};

	// check for wheels
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->conn == BIGWHEEL) {
			robot->z += (_bigwheel_radius - _end_height/2);
			_radius = _bigwheel_radius;
			break;
		}
		else if (robot->conn[i]->conn == SMALLWHEEL) {
			robot->z += (_smallwheel_radius - _end_height/2);
			_radius = _smallwheel_radius;
			break;
		}
		else if (robot->conn[i]->conn == WHEEL) {
			robot->z += (robot->conn[i]->size - _body_height/2);
			_radius = robot->conn[i]->size;
			if (fabs(robot->z) > (_body_radius-EPSILON)) { robot->z += _body_height/2; }
			break;
		}
	}

	// build robot
	double rot[4] = {robot->angle1, robot->angle2, robot->angle3, robot->angle4};
	this->buildIndividual(robot->x, robot->y, robot->z, R, rot);

	// add connectors
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->robot == _id)
			this->addConnector(robot->conn[i]->type, robot->conn[i]->face1, robot->conn[i]->size);
	}

	// set trackwidth
	double wheel[4] = {0};
	const double *pos;
	int i = 0;
	for (int i = 0; i < _conn.size(); i++) {
		switch (_conn[i]->type) {
			case BIGWHEEL:
			case SMALLWHEEL:
			case TINYWHEEL:
			case WHEEL:
				pos = dBodyGetPosition(_conn[i]->body);
				wheel[i++] = pos[0];
				wheel[i++] = pos[1];
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

int CMobot::build(XMLRobot *robot, dMatrix3 R, double *m, dBodyID base, XMLConn *conn) {
	// initialize new variables
	int i = 1;
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, R5, R6;

	// generate parameters for connector
	this->getConnectorParams(conn->type, conn->side, R, m);

	// collect data
	double r_le = robot->angle1;
	double r_lb = robot->angle2;
	double r_rb = robot->angle3;
	double r_re = robot->angle4;

	switch (conn->face2) {
		case 1:
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], 0);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], DEG2RAD(r_le));
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], -DEG2RAD(r_lb));
			dMultiply0(R6, R5, R4, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_lb));
			offset[0] = _end_depth + _body_end_depth + _body_length + R1[0]*_body_radius;
			offset[1] = R1[4]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 2: case 5:
			i = (conn->face2 == 2) ? -1 : 1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], i*M_PI/2);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], -DEG2RAD(r_lb));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_lb));
			offset[0] = _body_width/2;
			offset[1] = i*_body_end_depth + i*_body_length - i*_body_mount_center + i*R1[0]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 3: case 6:
			i = (conn->face2 == 3) ? -1 : 1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], i*M_PI/2);
			dMultiply0(R6, R1, R, 3, 3, 3);
			// center offset
			offset[0] = _body_width/2;
			break;
		case 4: case 7:
			i = (conn->face2 == 4) ? 1 : -1;
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], -i*M_PI/2);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[1], R2[5], R2[9], DEG2RAD(r_rb));
			dMultiply0(R6, R3, R2, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_rb));
			offset[0] = _body_width/2;
			offset[1] = i*_body_end_depth + i*_body_length - i*_body_mount_center + i*R1[0]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
		case 8:
			// rotation matrix
			dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI);
			dMultiply0(R2, R1, R, 3, 3, 3);
			dRFromAxisAndAngle(R3, R2[0], R2[4], R2[8], -DEG2RAD(r_re));
			dMultiply0(R4, R3, R2, 3, 3, 3);
			dRFromAxisAndAngle(R5, R4[1], R4[5], R4[9], DEG2RAD(r_rb));
			dMultiply0(R6, R5, R4, 3, 3, 3);
			// center offset
			dRFromAxisAndAngle(R1, 0, 1, 0, -DEG2RAD(r_rb));
			offset[0] = _end_depth + _body_end_depth + _body_length + R1[0]*_body_radius;
			offset[1] = R1[4]*_body_radius;
			offset[2] = R1[8]*_body_radius;
			break;
	}

	// adjust position by rotation matrix
	m[0] += R[0]*offset[0] + R[1]*offset[1] + R[2]*offset[2];
	m[1] += R[4]*offset[0] + R[5]*offset[1] + R[6]*offset[2];
	m[2] += R[8]*offset[0] + R[9]*offset[1] + R[10]*offset[2];

    // build new module
	double rot[4] = {r_le, r_lb, r_rb, r_re};
	this->buildIndividual(m[0], m[1], m[2], R6, rot);

    // add fixed joint to attach two modules
	this->fixBodyToConnector(base, conn->face2);

	// add connectors
	for (int i = 0; i < robot->conn.size(); i++) {
		if (robot->conn[i]->robot == _id)
			this->addConnector(robot->conn[i]->type, robot->conn[i]->face1, robot->conn[i]->size);
	}

	// fix to ground
	if (robot->ground != -1) this->fixBodyToGround(_body[robot->ground]);

	// success
	return 0;
}

int CMobot::buildIndividual(double x, double y, double z, dMatrix3 R, double *rot) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) { _body[i] = dBodyCreate(_world); }
	_geom[ENDCAP_L] = new dGeomID[7];
	_geom[BODY_L] = new dGeomID[5];
	_geom[CENTER] = new dGeomID[3];
	_geom[BODY_R] = new dGeomID[5];
	_geom[ENDCAP_R] = new dGeomID[7];

	// adjust input height by body height
	if (fabs(z) < (_end_height/2 - EPSILON)) {
		x += R[2]*_end_height/2;
		y += R[6]*_end_height/2;
		z += R[10]*_end_height/2;
	}

    // input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = DEG2RAD(rot[i]);
	}

	// offset values for each body part[0-2] and joint[3-5] from center
	double le[6] = {-_body_radius - _body_length - _body_end_depth - _end_depth/2, 0, 0, -_body_radius/2 - _body_length - _body_end_depth, 0, 0};
	double lb[6] = {-_body_radius - _body_length - _body_end_depth/2, 0, 0, -_center_length/2, _center_width/2, 0};
	double ce[3] = {0, _center_offset, 0};
	double rb[6] = {_body_radius + _body_length + _body_end_depth/2, 0, 0, _center_length/2, _center_width/2, 0};
	double re[6] = {_body_radius + _body_length + _body_end_depth + _end_depth/2, 0, 0, _body_radius/2 + _body_length + _body_end_depth, 0, 0};

	// build robot bodies
	this->build_endcap(ENDCAP_L, R[0]*le[0] + x, R[4]*le[0] + y, R[8]*le[0] + z, R);
	this->build_body(BODY_L, R[0]*lb[0] + x, R[4]*lb[0] + y, R[8]*lb[0] + z, R, 0);
	this->build_center(R[1]*ce[1] + x, R[5]*ce[1] + y, R[9]*ce[1] + z, R);
	this->build_body(BODY_R, R[0]*rb[0] + x, R[4]*rb[0] + y, R[8]*rb[0] + z, R, 0);
	this->build_endcap(ENDCAP_R, R[0]*re[0] + x, R[4]*re[0] + y, R[8]*re[0] + z, R);

	// get center of robot offset from body position
	_center[0] = 0;
	_center[1] = -0.0149;
	_center[2] = 0;

	// joint for left endcap to body
	_joint[0] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[0], _body[BODY_L], _body[ENDCAP_L]);
	dJointSetHingeAnchor(_joint[0], R[0]*le[3] + R[1]*le[4] + R[2]*le[5] + x,
									R[4]*le[3] + R[5]*le[4] + R[6]*le[5] + y,
									R[8]*le[3] + R[9]*le[4] + R[10]*le[5] + z);
	dJointSetHingeAxis(_joint[0], R[0], R[4], R[8]);

	// joint for center to left body 1
	_joint[1] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[1], _body[CENTER], _body[BODY_L]);
	dJointSetHingeAnchor(_joint[1], R[0]*lb[3] + R[1]*(_center_offset+lb[4]) + R[2]*lb[5] + x,
									R[4]*lb[3] + R[5]*(_center_offset+lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset+lb[4]) + R[10]*lb[5] + z);
	dJointSetHingeAxis(_joint[1], -R[1], -R[5], -R[9]);

	// joint for center to left body 2
	_joint[4] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[4], _body[CENTER], _body[BODY_L]);
	dJointSetHingeAnchor(_joint[4], R[0]*lb[3] + R[1]*(_center_offset-lb[4]) + R[2]*lb[5] + x,
									R[4]*lb[3] + R[5]*(_center_offset-lb[4]) + R[6]*lb[5] + y,
									R[8]*lb[3] + R[9]*(_center_offset-lb[4]) + R[10]*lb[5] + z);
	dJointSetHingeAxis(_joint[4], R[1], R[5], R[9]);

	// joint for center to right body 1
	_joint[2] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[2], _body[CENTER], _body[BODY_R]);
	dJointSetHingeAnchor(_joint[2], R[0]*rb[3] + R[1]*(_center_offset+rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset+rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset+rb[4]) + R[10]*rb[5] + z);
	dJointSetHingeAxis(_joint[2], -R[1], -R[5], -R[9]);

	// joint for center to right body 2
	_joint[5] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[5], _body[CENTER], _body[BODY_R]);
	dJointSetHingeAnchor(_joint[5], R[0]*rb[3] + R[1]*(_center_offset-rb[4]) + R[2]*rb[5] + x,
									R[4]*rb[3] + R[5]*(_center_offset-rb[4]) + R[6]*rb[5] + y,
									R[8]*rb[3] + R[9]*(_center_offset-rb[4]) + R[10]*rb[5] + z);
	dJointSetHingeAxis(_joint[5], R[1], R[5], R[9]);

	// joint for right body to endcap
	_joint[3] = dJointCreateHinge(_world, 0);
	dJointAttach(_joint[3], _body[BODY_R], _body[ENDCAP_R]);
	dJointSetHingeAnchor(_joint[3], R[0]*re[3] + R[1]*re[4] + R[2]*re[5] + x,
									R[4]*re[3] + R[5]*re[4] + R[6]*re[5] + y,
									R[8]*re[3] + R[9]*re[4] + R[10]*re[5] + z);
	dJointSetHingeAxis(_joint[3], R[0], R[4], R[8]);

	// create rotation matrices for each body part
	dMatrix3 R_e, R_b, R_le, R_lb, R_rb, R_re;
	dRFromAxisAndAngle(R_b, 0, 1, 0, _motor[JOINT1].theta);
	dMultiply0(R_lb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, -1, 0, 0, _motor[JOINT2].theta);
	dMultiply0(R_le, R_lb, R_e, 3, 3, 3);
	dRFromAxisAndAngle(R_b, 0, 1, 0, _motor[JOINT3].theta);
	dMultiply0(R_rb, R, R_b, 3, 3, 3);
	dRFromAxisAndAngle(R_e, -1, 0, 0, _motor[JOINT4].theta);
	dMultiply0(R_re, R_rb, R_e, 3, 3, 3);

	// if bodies are rotated, then redraw
	if (_motor[JOINT1].theta != 0 || _motor[JOINT2].theta != 0 || _motor[JOINT3].theta != 0 || _motor[JOINT4].theta != 0 ) {
		// offset values from center of robot
		double le_r[3] = {-_body_radius - (_body_length + _body_end_depth + _end_depth/2)*cos(_motor[JOINT2].theta), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_motor[JOINT2].theta)};
		double lb_r[3] = {-_body_radius - (_body_length + _body_end_depth/2)*cos(_motor[JOINT2].theta), 0, (_body_length + _body_end_depth/2)*sin(_motor[JOINT2].theta)};
		double rb_r[3] = {_body_radius + (_body_length + _body_end_depth/2)*cos(_motor[JOINT3].theta), 0, (_body_length + _body_end_depth/2)*sin(_motor[JOINT3].theta)};
		double re_r[3] = {_body_radius + (_body_length + _body_end_depth + _end_depth/2)*cos(_motor[JOINT3].theta), 0, (_body_length + _body_end_depth + _end_depth/2)*sin(_motor[JOINT3].theta)};
		// re-build pieces of module
		this->build_endcap(ENDCAP_L, R[0]*le_r[0] + R[2]*le_r[2] + x, R[4]*le_r[0] + R[6]*le_r[2] + y, R[8]*le_r[0] + R[10]*le_r[2] + z, R_le);
		this->build_body(BODY_L, R[0]*lb_r[0] + R[2]*lb_r[2] + x, R[4]*lb_r[0] + R[6]*lb_r[2] + y, R[8]*lb_r[0] + R[10]*lb_r[2] + z, R_lb, rot[JOINT2]);
		this->build_body(BODY_R, R[0]*rb_r[0] + R[2]*rb_r[2] + x, R[4]*rb_r[0] + R[6]*rb_r[2] + y, R[8]*rb_r[0] + R[10]*rb_r[2] + z, R_rb, rot[JOINT3]);
		this->build_endcap(ENDCAP_R, R[0]*re_r[0] + R[2]*re_r[2] + x, R[4]*re_r[0] + R[6]*re_r[2] + y, R[8]*re_r[0] + R[10]*re_r[2] + z, R_re);
	}

	// motor for left endcap to body
	_motor[JOINT1].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT1].id, _body[BODY_L], _body[ENDCAP_L]);
	dJointSetAMotorMode(_motor[JOINT1].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT1].id, 1);
	dJointSetAMotorAxis(_motor[JOINT1].id, 0, 1, R_lb[0], R_lb[4], R_lb[8]);
	dJointSetAMotorAngle(_motor[JOINT1].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT1].id, dParamFMax, _motor[JOINT1].tau_max);
	dJointDisable(_motor[JOINT1].id);

	// motor for center to left body
	_motor[JOINT2].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT2].id, _body[CENTER], _body[BODY_L]);
	dJointSetAMotorMode(_motor[JOINT2].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT2].id, 1);
	dJointSetAMotorAxis(_motor[JOINT2].id, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(_motor[JOINT2].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT2].id, dParamFMax, _motor[JOINT2].tau_max);
	dJointDisable(_motor[JOINT2].id);

	// motor for center to right body
	_motor[JOINT3].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT3].id, _body[CENTER], _body[BODY_R]);
	dJointSetAMotorMode(_motor[JOINT3].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT3].id, 1);
	dJointSetAMotorAxis(_motor[JOINT3].id, 0, 1, -R[1], -R[5], -R[9]);
	dJointSetAMotorAngle(_motor[JOINT3].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT3].id, dParamFMax, _motor[JOINT3].tau_max);
	dJointDisable(_motor[JOINT3].id);

	// motor for right body to endcap
	_motor[JOINT4].id = dJointCreateAMotor(_world, 0);
	dJointAttach(_motor[JOINT4].id, _body[BODY_R], _body[ENDCAP_R]);
	dJointSetAMotorMode(_motor[JOINT4].id, dAMotorUser);
	dJointSetAMotorNumAxes(_motor[JOINT4].id, 1);
	dJointSetAMotorAxis(_motor[JOINT4].id, 0, 1, R_rb[0], R_rb[4], R_rb[8]);
	dJointSetAMotorAngle(_motor[JOINT4].id, 0, 0);
	dJointSetAMotorParam(_motor[JOINT4].id, dParamFMax, _motor[JOINT4].tau_max);
	dJointDisable(_motor[JOINT4].id);

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// success
	return 0;
}

int CMobot::fixBodyToConnector(dBodyID cBody, int face) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	switch (face) {
		case 1:
			dJointAttach(joint, cBody, this->getBodyID(ENDCAP_L));
			break;
		case 2: case 5:
			dJointAttach(joint, cBody, this->getBodyID(BODY_L));
			break;
		case 3: case 6:
			dJointAttach(joint, cBody, this->getBodyID(BODY_L));
			dJointAttach(joint, cBody, this->getBodyID(BODY_R));
			break;
		case 4: case 7:
			dJointAttach(joint, cBody, this->getBodyID(BODY_R));
			break;
		case 8:
			dJointAttach(joint, cBody, this->getBodyID(ENDCAP_R));
			break;
	}

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int CMobot::fixConnectorToBody(int face, dBodyID cBody, int conn) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);
	dJointID joint2 = dJointCreateFixed(_world, 0);

	// attach to correct body
	switch (face) {
		case 1:
			dJointAttach(joint, this->getBodyID(ENDCAP_L), cBody);
			break;
		case 2: case 5:
			dJointAttach(joint, this->getBodyID(BODY_L), cBody);
			break;
		case 3: case 6:
			dJointAttach(joint, this->getBodyID(BODY_L), cBody);
			dJointAttach(joint2, this->getBodyID(BODY_R), cBody);
			dJointSetFixed(joint2);
			break;
		case 4: case 7:
			dJointAttach(joint, this->getBodyID(BODY_R), cBody);
			break;
		case 8:
			dJointAttach(joint, this->getBodyID(ENDCAP_R), cBody);
			break;
	}

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

double CMobot::getAngle(int id) {
	if (id == JOINT2 || id == JOINT3)
		_motor[id].theta = dJointGetHingeAngle(_joint[id]);
	else
		_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_joint[id]), dJointGetHingeAngleRate(_joint[id])) - _motor[id].offset;

    return _motor[id].theta;
}

int CMobot::getConnectorParams(int type, int side, dMatrix3 R, double *p) {
	double offset[3] = {0};
	dMatrix3 R1, R2, R3, R4, Rtmp = {R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9], R[10], R[11]};

	switch (type) {
		case SIMPLE:
			offset[0] = _conn_depth;
			dRSetIdentity(R1);
			break;
		case SQUARE:
			if (side == 2) {
				offset[0] = _end_width/2;
				offset[1] = _end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], M_PI/2);
			}
			else if (side == 3) {
				offset[0] = _end_width;
				dRSetIdentity(R1);
			}
			else if (side == 4) {
				offset[0] = _end_width/2;
				offset[1] = -_end_width/2;
				dRFromAxisAndAngle(R1, R[2], R[6], R[10], -M_PI/2);
			}
			break;
		case TANK:
			if (side == 2) {
				offset[0] = _tank_depth;
				dRSetIdentity(R1);
			}
			else if (side == 3) {
				offset[0] = _tank_depth/2;
				offset[2] = _tank_height - _conn_height/2;
				dRFromAxisAndAngle(R1, R[1], R[5], R[9], -M_PI/2);
			}
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

int CMobot::getFaceParams(int face, dMatrix3 R, double *p) {
	const double *pos, *R1;
	dMatrix3 R2;
	double offset[3] = {0};
	int i = 1;

	switch (face) {
		case 1:
			pos = dBodyGetPosition(_body[ENDCAP_L]);
			R1 = dBodyGetRotation(_body[ENDCAP_L]);
			offset[0] = -_end_depth/2;
			p[0] = pos[0] + R1[0]*offset[0];
			p[1] = pos[1] + R1[4]*offset[0];
			p[2] = pos[2] + R1[8]*offset[0];
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 2: case 5:
			pos = dGeomGetPosition(_geom[BODY_L][0]);
			R1 = dBodyGetRotation(_body[BODY_L]);
			i = ((face == 5) ? 1 : -1);
			offset[0] = -_body_end_depth/2 + _body_mount_center;
			offset[1] = i*_body_width/2;
			p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
			p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 3: case 6:
			pos = dBodyGetPosition(_body[CENTER]);
			R1 = dBodyGetRotation(_body[CENTER]);
			i = (face == 6) ? 1 : -1;
			offset[1] = i*(_body_width/2) - _center_offset;
			p[0] = pos[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[5]*offset[1];
			p[2] = pos[2] + R1[9]*offset[1];
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 4: case 7:
			pos = dGeomGetPosition(_geom[BODY_R][0]);
			R1 = dBodyGetRotation(_body[BODY_R]);
			i = (face == 7) ? 1 : -1;
			offset[0] = _body_end_depth/2 - _body_mount_center;
			offset[1] = i*_body_width/2;
			p[0] = pos[0] + R1[0]*offset[0] + R1[1]*offset[1];
			p[1] = pos[1] + R1[4]*offset[0] + R1[5]*offset[1];
			p[2] = pos[2] + R1[8]*offset[0] + R1[9]*offset[1];
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], i*M_PI/2);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
		case 8:
			pos = dBodyGetPosition(_body[ENDCAP_R]);
			R1 = dBodyGetRotation(_body[ENDCAP_R]);
			offset[0] = _end_depth/2;
			p[0] = pos[0] + R1[0]*offset[0];
			p[1] = pos[1] + R1[4]*offset[0];
			p[2] = pos[2] + R1[8]*offset[0];
			dRFromAxisAndAngle(R2, R1[2], R1[6], R1[10], 0);
			dMultiply0(R, R2, R1, 3, 3, 3);
			break;
	}

	// success
	return 0;
}

int CMobot::initParams(int disabled, int type) {
	_dof = 4;

	// create arrays for mobots
	_body = new dBodyID[NUM_PARTS];
	_enabled = new int[2];
	_geom = new dGeomID * [NUM_PARTS];
	_joint = new dJointID[6];
	_motor.resize(_dof);
	_neighbor.resize(_dof);

	// fill with default data
	for (int i = 0; i < _dof; i++) {
		_motor[i].alpha = 0;
		_motor[i].encoder = DEG2RAD(0.1);
		_motor[i].goal = 0;
		_motor[i].mode = SEEK;
		_motor[i].offset = 0;
		_motor[i].omega = 0.7854;			//  45 deg/sec
		_motor[i].omega_max = 2.0943;		// 120 deg/sec
		_motor[i].record = false;
		_motor[i].record_active = false;
		_motor[i].record_angle = new double * [_dof];
		_motor[i].record_num = 0;
		_motor[i].safety_angle = 10;
		_motor[i].safety_timeout = 4;
		_motor[i].state = NEUTRAL;
		_motor[i].success = true;
		_motor[i].theta = 0;
		_motor[i].timeout = 0;
		MUTEX_INIT(&_motor[i].success_mutex);
		COND_INIT(&_motor[i].success_cond);
	}
	_disabled = disabled;
	_distOffset = 0;
	_g_shift_data = 0;
	_g_shift_data_en = 0;
	_id = -1;
	_motor[JOINT1].tau_max = 0.260;
	_motor[JOINT2].tau_max = 1.059;
	_motor[JOINT3].tau_max = 1.059;
	_motor[JOINT4].tau_max = 0.260;
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

int CMobot::initDims(void) {
	_center_length = 0.0516;
	_center_width = 0.0327;
	_center_height = 0.0508;
	_center_radius = 0.0254;
	_center_offset = 0.0149;
	_body_length = 0.0258;
	_body_width = 0.0762;
	_body_height = 0.0508;
	_body_radius = 0.0254;
	_body_inner_width_left = 0.0366;
	_body_inner_width_right = 0.0069;
	_body_end_depth = 0.0352;
	_body_mount_center = 0.0374;
	_end_width = 0.0762;
	_end_height = 0.0762;
	_end_depth = 0.0080;
	_end_radius = 0.0254;
	_conn_depth = 0.0048;
	_conn_height = 0.0413;
	_conn_radius = 0.0064;
	_bigwheel_radius = 0.0571;
	_smallwheel_radius = 0.0445;
	_tank_depth = 0.0413;
	_tank_height = 0.0460;
	_wheel_radius = 0.0445;

	// success
	return 0;
}

void CMobot::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// get body rotation from world
	const double *R = dBodyGetRotation(_body[CENTER]);
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
		// set motor angle to current angle
		dJointSetAMotorAngle(_motor[i].id, 0, _motor[i].theta);
		// drive motor to get current angle to match future angle
		switch (_motor[i].mode) {
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
				if (_motor[i].theta < _motor[i].goal - _motor[i].encoder) {
					_motor[i].state = POSITIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, fabs(_motor[i].omega));
				}
				else if (_motor[i].theta > _motor[i].goal + _motor[i].encoder) {
					_motor[i].state = NEGATIVE;
					dJointSetAMotorParam(_motor[i].id, dParamVel, -fabs(_motor[i].omega));
				}
				else {
					_motor[i].state = HOLD;
					dJointSetAMotorParam(_motor[i].id, dParamVel, 0);
				}
				break;
		}
	}

	// unlock angle and goal
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

void CMobot::simPostCollisionThread(void) {
	// check if joint speed is zero -> joint has completed step
	for (int i = 0; i < _dof; i++) {
		// lock mutex
		MUTEX_LOCK(&_motor[i].success_mutex);
		// zero velocity == stopped
		_motor[i].success = (!(int)(dJointGetAMotorParam(this->_motor[i].id, dParamVel)*1000) );
		// signal success
		if (_motor[i].success)
			COND_SIGNAL(&_motor[i].success_cond);
		// unlock mutex
		MUTEX_UNLOCK(&_motor[i].success_mutex);
	}

	if (_motor[JOINT1].success && _motor[JOINT2].success && _motor[JOINT3].success && _motor[JOINT4].success)
		COND_SIGNAL(&_success_cond);
}

/**********************************************************
	private functions
 **********************************************************/
int CMobot::build_bigwheel(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];


    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_conn_depth/3, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _conn_depth/2, _end_width, _conn_height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry
    conn->geom[0] = dCreateCylinder(_space, _bigwheel_radius, 2*_conn_depth/3);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
    dGeomSetOffsetRotation(conn->geom[0], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_body(int id, double x, double y, double z, dMatrix3 R, double theta) {
	int i = 1;
	if ( id == BODY_R )
		i = -1;

    // define parameters
    dMass m, m1, m2, m3;
    dMatrix3 R1, R2, R3;

    // set mass of body
    dMassSetBox(&m1, 1000, _body_end_depth, _center_height, _body_width);
    dMassAdd(&m, &m1);
    dMassSetBox(&m2, 1000, _body_inner_width_left, _end_depth, _body_width);
    dMassTranslate(&m2, 0.01524*i, -0.0346, 0);
    dMassAdd(&m, &m2);
    dMassSetBox(&m3, 1000, _body_inner_width_right, _end_depth, _body_width);
    dMassTranslate(&m3, 0.01524*i, 0.0346, 0);
    dMassAdd(&m, &m3);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[id], x, y, z);
    dBodySetRotation(_body[id], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);
    dRFromAxisAndAngle(R3, 0, 0, 1, -theta);
    dMultiply0(R2, R1, R3, 3, 3, 3);

    // set geometry 1 - face
    _geom[id][0] = dCreateBox(_space, _body_end_depth, _body_width, _body_height);
    dGeomSetBody(_geom[id][0], _body[id]);
    dGeomSetOffsetPosition(_geom[id][0], -m.c[0], -m.c[1], -m.c[2]);
    // set geometry 2 - side square
    _geom[id][1] = dCreateBox( _space, _body_length, _body_inner_width_left, _body_height);
    dGeomSetBody( _geom[id][1], _body[id]);
    dGeomSetOffsetPosition( _geom[id][1], i*_body_length/2 + i*_body_end_depth/2 - m.c[0], -_body_width/2 + _body_inner_width_left/2 - m.c[1], -m.c[2] );
    // set geometry 3 - side square
    _geom[id][2] = dCreateBox( _space, _body_length, _body_inner_width_right, _body_height);
    dGeomSetBody( _geom[id][2], _body[id]);
    dGeomSetOffsetPosition( _geom[id][2], i*_body_length/2 + i*_body_end_depth/2 - m.c[0], _body_width/2 - _body_inner_width_right/2 - m.c[1], -m.c[2] );
    // set geometry 4 - side curve
    _geom[id][3] = dCreateCylinder( _space, _body_radius, _body_inner_width_left);
    dGeomSetBody( _geom[id][3], _body[id]);
    dGeomSetOffsetPosition( _geom[id][3], i*_body_length + i*_body_end_depth/2 - m.c[0], -_body_width/2 + _body_inner_width_left/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[id][3], R2);
    // set geometry 5 - side curve
    _geom[id][4] = dCreateCylinder( _space, _body_radius, _body_inner_width_right);
    dGeomSetBody( _geom[id][4], _body[id]);
    dGeomSetOffsetPosition( _geom[id][4], i*_body_length + i*_body_end_depth/2 - m.c[0], _body_width/2 - _body_inner_width_right/2 - m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[id][4], R2);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CMobot::build_caster(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[10];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double	depth = _conn_depth,
			width = _end_width,
			height = _conn_height,
			radius = _conn_radius,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

	// set mass of body
	dMassSetBox(&m, 1000, 2*_conn_depth, width, height);
	dMassTranslate(&m, 12*_conn_depth, 0, -height);

    // adjust x,y,z to position center of mass correctly
	p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width - 2*radius, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], -m.c[0], -width/2 + radius/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], -m.c[0], width/2 - radius/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], -m.c[0], -width/2 + radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[3], R1);

    // set geometry 5 - fillet upper right
    conn->geom[4] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[4], conn->body);
    dGeomSetOffsetPosition(conn->geom[4], -m.c[0], width/2 - radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[4], R1);

    // set geometry 6 - fillet lower right
    conn->geom[5] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[5], conn->body);
    dGeomSetOffsetPosition(conn->geom[5], -m.c[0], width/2 - radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[5], R1);

    // set geometry 7 - fillet lower left
    conn->geom[6] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[6], conn->body);
    dGeomSetOffsetPosition(conn->geom[6], -m.c[0], -width/2 + radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[6], R1);

    // set geometry 8 - horizontal support
	conn->geom[7] = dCreateBox(_space, 0.0667, 0.0222, 0.0032);
	dGeomSetBody(conn->geom[7], conn->body);
	dGeomSetOffsetPosition(conn->geom[7], depth/2 + 0.0667/2 - m.c[0], -m.c[1], -height/2 + 0.0016 - m.c[2]);

    // set geometry 9 - ball support
    conn->geom[8] = dCreateCylinder(_space, 0.0111, 0.0191);
    dGeomSetBody(conn->geom[8], conn->body);
    dGeomSetOffsetPosition(conn->geom[8], depth/2 + 0.0667 - m.c[0], -m.c[1], -height/2 - 0.0064 - m.c[2]);

    // set geometry 10 - sphere
    conn->geom[9] = dCreateSphere(_space, 0.0095);
    dGeomSetBody(conn->geom[9], conn->body);
    dGeomSetOffsetPosition(conn->geom[9], depth/2 + 0.0667 - m.c[0], -m.c[1], -height/2 - 0.0159 - m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_center(double x, double y, double z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetCapsule(&m, 270, 1, _center_radius, _center_length);

    // adjsut x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[CENTER], x, y, z);
    dBodySetRotation(_body[CENTER], R);

    // rotation matrix for curves of d-shapes
    dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2);

    // set geometry 1 - center rectangle
    _geom[CENTER][0] = dCreateBox(_space, _center_length, _center_width, _center_height );
    dGeomSetBody( _geom[CENTER][0], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - side curve
    _geom[CENTER][1] = dCreateCylinder(_space, _center_radius, _center_width );
    dGeomSetBody( _geom[CENTER][1], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][1], -_center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[CENTER][1], R1);

    // set geometry 3 - side curve
    _geom[CENTER][2] = dCreateCylinder(_space, _center_radius, _center_width );
    dGeomSetBody( _geom[CENTER][2], _body[CENTER]);
    dGeomSetOffsetPosition( _geom[CENTER][2], _center_length/2 - m.c[0], -m.c[1], -m.c[2] );
    dGeomSetOffsetRotation( _geom[CENTER][2], R1);

    // set mass center to (0,0,0) of body
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[CENTER], &m);

	// success
	return 0;
}

int CMobot::build_endcap(int id, double x, double y, double z, dMatrix3 R) {
    // define parameters
    dMass m;
    dMatrix3 R1;

    // set mass of body
    dMassSetBox(&m, 1000, _end_depth, _end_width, _end_height);

    // adjust x,y,z to position center of mass correctly
    x += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    y += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    z += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(_body[id], x, y, z);
    dBodySetRotation(_body[id], R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    _geom[id][0] = dCreateBox(_space, _end_depth, _end_width - 2*_end_radius, _end_height );
    dGeomSetBody( _geom[id][0], _body[id]);
    dGeomSetOffsetPosition( _geom[id][0], -m.c[0], -m.c[1], -m.c[2] );

    // set geometry 2 - left box
    _geom[id][1] = dCreateBox(_space, _end_depth, _end_radius, _end_height - 2*_end_radius );
    dGeomSetBody( _geom[id][1], _body[id]);
    dGeomSetOffsetPosition( _geom[id][1], -m.c[0], -_end_width/2 + _end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 3 - right box
    _geom[id][2] = dCreateBox(_space, _end_depth, _end_radius, _end_height - 2*_end_radius );
    dGeomSetBody( _geom[id][2], _body[id]);
    dGeomSetOffsetPosition( _geom[id][2], -m.c[0], _end_width/2 - _end_radius/2 - m.c[1], -m.c[2] );

    // set geometry 4 - fillet upper left
    _geom[id][3] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][3], _body[id]);
    dGeomSetOffsetPosition( _geom[id][3], -m.c[0], -_end_width/2 + _end_radius - m.c[1], _end_width/2 - _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][3], R1);

    // set geometry 5 - fillet upper right
    _geom[id][4] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][4], _body[id]);
    dGeomSetOffsetPosition( _geom[id][4], -m.c[0], _end_width/2 - _end_radius - m.c[1], _end_width/2 - _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][4], R1);

    // set geometry 6 - fillet lower right
    _geom[id][5] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][5], _body[id]);
    dGeomSetOffsetPosition( _geom[id][5], -m.c[0], _end_width/2 - _end_radius - m.c[1], -_end_width/2 + _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][5], R1);

    // set geometry 7 - fillet lower left
    _geom[id][6] = dCreateCylinder(_space, _end_radius, _end_depth );
    dGeomSetBody( _geom[id][6], _body[id]);
    dGeomSetOffsetPosition( _geom[id][6], -m.c[0], -_end_width/2 + _end_radius - m.c[1], -_end_width/2 + _end_radius - m.c[2] );
    dGeomSetOffsetRotation( _geom[id][6], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(_body[id], &m);

	// success
	return 0;
}

int CMobot::build_simple(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[7];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double	depth = _conn_depth,
			width = _end_width,
			height = _conn_height,
			radius = _conn_radius,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, depth, width, height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width - 2*radius, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], -m.c[0], -width/2 + radius/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, depth, radius, height - 2*radius);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], -m.c[0], width/2 - radius/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], -m.c[0], -width/2 + radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[3], R1);

    // set geometry 5 - fillet upper right
    conn->geom[4] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[4], conn->body);
    dGeomSetOffsetPosition(conn->geom[4], -m.c[0], width/2 - radius - m.c[1], height/2 - radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[4], R1);

    // set geometry 6 - fillet lower right
    conn->geom[5] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[5], conn->body);
    dGeomSetOffsetPosition(conn->geom[5], -m.c[0], width/2 - radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[5], R1);

    // set geometry 7 - fillet lower left
    conn->geom[6] = dCreateCylinder(_space, radius, depth);
    dGeomSetBody(conn->geom[6], conn->body);
    dGeomSetOffsetPosition(conn->geom[6], -m.c[0], -width/2 + radius - m.c[1], -height/2 + radius - m.c[2]);
    dGeomSetOffsetRotation(conn->geom[6], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_smallwheel(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

    // define parameters
    dMass m;
    dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_conn_depth/3, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _conn_depth/2, _end_width, _conn_height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry
    conn->geom[0] = dCreateCylinder(_space, _smallwheel_radius, 2*_conn_depth/3);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
    dGeomSetOffsetRotation(conn->geom[0], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_square(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[4];

    // define parameters
    dMass m;
    dMatrix3 R;
	double p[3] = {0}, offset[3] = {_conn_depth/2, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _end_width, _end_width, _conn_height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, _conn_depth, _end_width, _conn_height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);

    // set geometry 2 - left box
    conn->geom[1] = dCreateBox(_space, _end_width - 2*_conn_depth, _conn_depth, _conn_height);
    dGeomSetBody(conn->geom[1], conn->body);
    dGeomSetOffsetPosition(conn->geom[1], _end_width/2 - _conn_depth/2 - m.c[0], -_end_width/2 + _conn_depth/2 - m.c[1], -m.c[2]);

    // set geometry 3 - right box
    conn->geom[2] = dCreateBox(_space, _end_width - 2*_conn_depth, _conn_depth, _conn_height);
    dGeomSetBody(conn->geom[2], conn->body);
    dGeomSetOffsetPosition(conn->geom[2], _end_width/2 - _conn_depth/2 - m.c[0], _end_width/2 - _conn_depth/2 - m.c[1], -m.c[2]);

    // set geometry 4 - fillet upper left
    conn->geom[3] = dCreateBox(_space, _conn_depth, _end_width, _conn_height);
    dGeomSetBody(conn->geom[3], conn->body);
    dGeomSetOffsetPosition(conn->geom[3], _end_width - _conn_depth - m.c[0], -m.c[1], -m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_tank(Connector *conn, int face) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

    // define parameters
    dMass m;
    dMatrix3 R;
	double	depth = _tank_depth,
			width = _end_width,
			height = _tank_height,
			p[3] = {0},
			offset[3] = {depth/2, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, depth, width, height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // set geometry 1 - center box
    conn->geom[0] = dCreateBox(_space, depth, width, height);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], (_tank_height - _conn_height)/2 - m.c[2]);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

int CMobot::build_wheel(Connector *conn, int face, double size) {
	// create body
	conn->body = dBodyCreate(_world);
    conn->geom = new dGeomID[1];

	// store wheel radius
	_wheel_radius = size;

    // define parameters
	dMass m;
	dMatrix3 R, R1;
	double p[3] = {0}, offset[3] = {_conn_depth/3, 0, 0};

	// position center of connector
	this->getFaceParams(face, R, p);
	p[0] += R[0]*offset[0];
	p[1] += R[4]*offset[0];
	p[2] += R[8]*offset[0];

    // set mass of body
    dMassSetBox(&m, 270, _conn_depth/2, _end_width, _conn_height);

    // adjust x,y,z to position center of mass correctly
    p[0] += R[0]*m.c[0] + R[1]*m.c[1] + R[2]*m.c[2];
    p[1] += R[4]*m.c[0] + R[5]*m.c[1] + R[6]*m.c[2];
    p[2] += R[8]*m.c[0] + R[9]*m.c[1] + R[10]*m.c[2];

    // set body parameters
    dBodySetPosition(conn->body, p[0], p[1], p[2]);
    dBodySetRotation(conn->body, R);

    // rotation matrix for curves
    dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2);

    // set geometry
    conn->geom[0] = dCreateCylinder(_space, _wheel_radius, 2*_conn_depth/3);
    dGeomSetBody(conn->geom[0], conn->body);
    dGeomSetOffsetPosition(conn->geom[0], -m.c[0], -m.c[1], -m.c[2]);
    dGeomSetOffsetRotation(conn->geom[0], R1);

    // set mass center to (0,0,0) of _bodyID
    dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    dBodySetMass(conn->body, &m);

	// fix connector to body
	this->fixConnectorToBody(face, conn->body);

	// success
	return 0;
}

