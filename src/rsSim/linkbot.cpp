#include <rsSim/sim.hpp>
#include <rsSim/linkbot.hpp>

using namespace rsSim;

CLinkbotT::CLinkbotT(int disabled) : rsRobots::Robot(rs::LINKBOTT), rsRobots::LinkbotT(rs::JOINT1), Robot(rs::JOINT1, rs::JOINT3) {
	// initialize parameters
	this->init_params();

	// disabled joint
	_disabled = disabled;
}

CLinkbotT::~CLinkbotT(void) {
	// remove robot from simulation
	if (!_sim->deleteRobot(_pos)) { delete _sim; }

	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}
}

int CLinkbotT::accelJointAngleNB(rs::JointID id, double a, double angle) {
	this->accelJointTimeNB(id, a, sqrt(2*angle/a));

	// success
	return 0;
}

int CLinkbotT::accelJointCycloidalNB(rs::JointID id, double angle, double t) {
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

int CLinkbotT::accelJointHarmonicNB(rs::JointID id, double angle, double t) {
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

int CLinkbotT::accelJointSmoothNB(rs::JointID id, double a0, double af, double vmax, double angle) {
	_motor[id].omega = DEG2RAD(vmax);
	this->moveJoint(id, angle);

	// success
	return 0;
}

int CLinkbotT::accelJointTimeNB(rs::JointID id, double a, double t) {
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

int CLinkbotT::accelJointToMaxSpeedNB(rs::JointID id, double a) {
	this->accelJointTimeNB(id, a, 0);

	// success
	return 0;
}

int CLinkbotT::accelJointToVelocityNB(rs::JointID id, double a, double v) {
	this->accelJointTimeNB(id, a, v/a);

	// success
	return 0;
}

int CLinkbotT::closeGripper(void) {
	double gripperAngleOld = 0;
	double gripperAngleNew = 0;
	int retval = getJointAngleInstant(rs::JOINT1, gripperAngleNew);
	while ( fabs(gripperAngleNew - gripperAngleOld) > 0.1 ) {
		gripperAngleOld = gripperAngleNew;
		retval = retval || getJointAngleInstant(rs::JOINT1, gripperAngleNew);
		retval = retval || moveNB(8, 0, 8);
		delaySeconds(1);
		retval = retval || getJointAngleInstant(rs::JOINT1, gripperAngleNew);
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
	this->accelJointCycloidalNB(rs::JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointCycloidalNB(rs::JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelDistanceNB(double radius, double a, double d) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(rs::JOINT1,  RAD2DEG(a/radius), sqrt(2*d/a));
	this->accelJointTimeNB(rs::JOINT3, -RAD2DEG(a/radius), sqrt(2*d/a));

	// success
	return 0;
}

int CLinkbotT::driveAccelHarmonicNB(double radius, double d, double t) {
	this->accelJointHarmonicNB(rs::JOINT1,  RAD2DEG(d/radius), t);
	this->accelJointHarmonicNB(rs::JOINT3, -RAD2DEG(d/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	this->accelJointSmoothNB(rs::JOINT1, a0, af, vmax, d/radius);
	this->accelJointSmoothNB(rs::JOINT3, a0, af, vmax, d/radius);

	// success
	return 0;
}

int CLinkbotT::driveAccelTimeNB(double radius, double a, double t) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(rs::JOINT1,  RAD2DEG(a/radius), t);
	this->accelJointTimeNB(rs::JOINT3, -RAD2DEG(a/radius), t);

	// success
	return 0;
}

int CLinkbotT::driveAccelToMaxSpeedNB(double radius, double a) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(rs::JOINT1,  RAD2DEG(a/radius), 0);
	this->accelJointTimeNB(rs::JOINT3, -RAD2DEG(a/radius), 0);

	// success
	return 0;
}

int CLinkbotT::driveAccelToVelocityNB(double radius, double a, double v) {
	a = DEG2RAD(a);
	this->accelJointTimeNB(rs::JOINT1,  RAD2DEG(a/radius), v/a);
	this->accelJointTimeNB(rs::JOINT3, -RAD2DEG(a/radius), v/a);

	// success
	return 0;
}

int CLinkbotT::driveForeverNB(void) {
	// negate speed to act as a car
	_motor[rs::JOINT3].omega = -_motor[rs::JOINT3].omega;

	// set joint movements
	this->moveJointForeverNB(rs::JOINT1);
	this->moveJointForeverNB(rs::JOINT3);

	// success
	return 0;
}

int CLinkbotT::driveForwardNB(double angle) {
	this->moveJointNB(rs::JOINT1, angle);
	this->moveJointNB(rs::JOINT3, -angle);

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
	this->setJointSpeed(rs::JOINT1, RAD2DEG(s1/theta/rho/radius*_speed));
	this->setJointSpeed(rs::JOINT3, RAD2DEG(s2/theta/rho/radius*_speed));
	this->moveJointNB(rs::JOINT1, RAD2DEG(s1/radius));
	this->moveJointNB(rs::JOINT3, -RAD2DEG(s2/radius));
	this->delay(theta*rho/_speed*1000);

	// success
	return 0;
}

int CLinkbotT::getJointAngles(double &angle1, double &angle2, double &angle3, int numReadings) {
	this->getJointAngle(rs::JOINT1, angle1, numReadings);
	this->getJointAngle(rs::JOINT2, angle2, numReadings);
	this->getJointAngle(rs::JOINT3, angle3, numReadings);

	// success
	return 0;
}

int CLinkbotT::getJointAnglesInstant(double &angle1, double &angle2, double &angle3) {
	this->getJointAngleInstant(rs::JOINT1, angle1);
	this->getJointAngleInstant(rs::JOINT2, angle2);
	this->getJointAngleInstant(rs::JOINT3, angle3);

	// success
	return 0;
}

int CLinkbotT::getJointSpeeds(double &speed1, double &speed2, double &speed3) {
	speed1 = RAD2DEG(_motor[rs::JOINT1].omega);
	speed2 = RAD2DEG(_motor[rs::JOINT2].omega);
	speed3 = RAD2DEG(_motor[rs::JOINT3].omega);

	// success
	return 0;
}

int CLinkbotT::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3) {
	ratio1 = _motor[rs::JOINT1].omega/_motor[rs::JOINT1].omega_max;
	ratio2 = _motor[rs::JOINT2].omega/_motor[rs::JOINT2].omega_max;
	ratio3 = _motor[rs::JOINT3].omega/_motor[rs::JOINT3].omega_max;

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
	angles[rs::JOINT1] = angle1;
	angles[rs::JOINT2] = angle2;
	angles[rs::JOINT3] = angle3;

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
	angles[rs::JOINT1] = angle1;
	angles[rs::JOINT2] = angle2;
	angles[rs::JOINT3] = angle3;

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
	if (_form == rs::LINKBOTL)
		this->moveJointToNB(rs::JOINT1, -angle);
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
	angles[rs::JOINT1] = angle1;
	angles[rs::JOINT2] = angle2;
	angles[rs::JOINT3] = angle3;

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
	angles[rs::JOINT1] = angle1;
	angles[rs::JOINT2] = angle2;
	angles[rs::JOINT3] = angle3;

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
	angles[rs::JOINT1] = distance1;
	angles[rs::JOINT2] = distance2;
	angles[rs::JOINT3] = distance3;

	// call base class recording function
	return Robot::recordAnglesBegin(time, angles, seconds, shiftData);
}

int CLinkbotT::setJointSpeeds(double speed1, double speed2, double speed3) {
	this->setJointSpeed(rs::JOINT1, speed1);
	this->setJointSpeed(rs::JOINT2, speed2);
	this->setJointSpeed(rs::JOINT3, speed3);

	// success
	return 0;
}

int CLinkbotT::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	this->setJointSpeedRatio(rs::JOINT1, ratio1);
	this->setJointSpeedRatio(rs::JOINT2, ratio2);
	this->setJointSpeedRatio(rs::JOINT3, ratio3);

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
int CLinkbotT::addConnector(int type, int face, double size, int side, int conn) {
	// get connector body position
	const double *pos = dBodyGetPosition(_body[BODY]);
	const double *quat = dBodyGetQuaternion(_body[BODY]);
	double p[3] = {pos[0], pos[1], pos[2]}, p1[3], p2[3];
	double q[4] = {quat[1], quat[2], quat[3], quat[0]}, q1[4], q2[4];
	this->getRobotFaceOffset(face, p, q, p1, q1);
	if (conn == -1)
		this->getConnBodyOffset(type, p1, q1, p2, q2);
	else {
		this->getConnFaceOffset(type, side, p1, q1, p2, q2);
		this->getConnBodyOffset(conn, p2, q2, p2, q2);
	}

	// create new connector
	_conn.push_back(new Connector());

	// daisy chained or not
	if (conn == -1) {
		_conn.back()->d_side = -1;
		_conn.back()->d_type = -1;
		_conn.back()->face = face;
		_conn.back()->type = type;
		_conn.back()->body = dBodyCreate(_world);
	}
	else {
		_conn.back()->d_side = side;
		_conn.back()->d_type = type;
		_conn.back()->face = face;
		_conn.back()->type = conn;
		_conn.back()->body = dBodyCreate(_world);
		type = conn;
	}

	// build connector
	switch (type) {
		case rs::BIGWHEEL:
			this->build_wheel(_conn.back(), _bigwheel_radius);
			break;
		case rs::BRIDGE:
			this->build_bridge(_conn.back());
			break;
		case rs::CASTER:
			this->build_caster(_conn.back(), static_cast<int>(size));
			break;
		case rs::CUBE:
			this->build_cube(_conn.back());
			break;
		case rs::FACEPLATE:
			this->build_faceplate(_conn.back());
			break;
		case rs::GRIPPER:
			this->build_gripper(_conn.back(), face);
			break;
		case rs::OMNIDRIVE:
			this->build_omnidrive(_conn.back());
			break;
		case rs::SIMPLE:
			this->build_simple(_conn.back());
			break;
		case rs::SMALLWHEEL:
			this->build_wheel(_conn.back(), _smallwheel_radius);
			break;
		case rs::TINYWHEEL:
			this->build_wheel(_conn.back(), _tinywheel_radius);
			break;
		case rs::WHEEL:
			this->build_wheel(_conn.back(), size);
			break;
	}

	if (type == rs::GRIPPER) {
		_conn.push_back(new Connector());
		_conn.back()->d_side = -1;
		_conn.back()->d_type = -1;
		_conn.back()->face = face;
		_conn.back()->type = type;
		this->build_gripper(_conn.back(), 3);
	}

	// set body parameters
	dBodySetPosition(_conn.back()->body, p2[0], p2[1], p2[2]);
	dQuaternion Q = {q2[3], q2[0], q2[1], q2[2]};
	dBodySetQuaternion(_conn.back()->body, Q);

	// fix connector to body
	this->fixConnectorToBody(face, _conn.back()->body, conn);

	// success
	return 0;
}

int CLinkbotT::build(int id, const double *p, const double *q, const double *a, int ground) {
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

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int CLinkbotT::build(int id, const double *p, const double *q, const double *a, dBodyID base, int face, int ground) {
	// get offset of robot
	double o[3], p1[3], p2[3] = {0};
	double q1[4], q2[4] = {0, 0, 0, 1};
	switch (face) {
		case FACE1:
			p2[0] = _body_width/2 + _face_depth;
			break;
		case FACE2:
			p2[1] = _face_depth + _body_length;
			q2[2] = sin(-0.785398);	// 0.5*PI/2
			q2[3] = cos(-0.785398);	// 0.5*PI/2
			break;
		case FACE3:
			p2[0] = _body_width/2 + _face_depth;
			q2[2] = sin(1.570796);	// 0.5*PI
			q2[3] = cos(1.570796);	// 0.5*PI
			break;
	}
	this->multiplyQbyV(q, p2[0], p2[1], p2[2], o);
	p1[0] = p[0] + o[0];
	p1[1] = p[1] + o[1];
	p1[2] = p[2] + o[2];
	this->multiplyQbyQ(q, q2, q1);

    // build new module
	this->buildIndividual(p1, q1, a);

    // add fixed joint to attach two modules
	this->fixBodyToConnector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int CLinkbotT::buildIndividual(const double *p, const double *q, const double *a) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// init geom arrays
	dGeomID **geom = new dGeomID * [NUM_PARTS];
	geom[BODY] = new dGeomID[2];
	geom[FACE1] = new dGeomID[1];
	geom[FACE2] = new dGeomID[1];
	geom[FACE3] = new dGeomID[1];

	// convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = DEG2RAD(a[i]);
	}

	// build robot bodies
	double p1[3], q1[4];
	this->build_body(geom[BODY], p, q);
	this->getRobotBodyOffset(FACE1, p, q, p1, q1);
	this->build_face(FACE1, geom[FACE1], p1, q1);
	this->getRobotBodyOffset(FACE2, p, q, p1, q1);
	this->build_face(FACE2, geom[FACE2], p1, q1);
	this->getRobotBodyOffset(FACE3, p, q, p1, q1);
	this->build_face(FACE3, geom[FACE3], p1, q1);

	// joint variable
	dJointID *joint = new dJointID[_dof];
	double o[3];

	// joint for body to face 1
	joint[rs::JOINT1] = dJointCreateHinge(_world, 0);
	dJointAttach(joint[rs::JOINT1], _body[BODY], _body[FACE1]);
	this->multiplyQbyV(q, -_body_width/2, 0, 0, o);
	dJointSetHingeAnchor(joint[rs::JOINT1], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	this->multiplyQbyV(q, 1, 0, 0, o);
	dJointSetHingeAxis(joint[rs::JOINT1], o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[FACE1], o[0], o[1], o[2]);

	// joint for body to face 2
	if (_disabled == rs::JOINT2) {
		joint[rs::JOINT2] = dJointCreateFixed(_world, 0);
		dJointAttach(joint[rs::JOINT2], _body[BODY], _body[FACE2]);
		dJointSetFixed(joint[rs::JOINT2]);
	}
	else {
		joint[rs::JOINT2] = dJointCreateHinge(_world, 0);
		dJointAttach(joint[rs::JOINT2], _body[BODY], _body[FACE2]);
		this->multiplyQbyV(q, 0, -_body_length, 0, o);
		dJointSetHingeAnchor(joint[rs::JOINT2], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		this->multiplyQbyV(q, 0, 1, 0, o);
		dJointSetHingeAxis(joint[rs::JOINT2], o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[FACE2], o[0], o[1], o[2]);
	}

	// joint for body to face 3
	if (_disabled == rs::JOINT3) {
		joint[rs::JOINT3] = dJointCreateFixed(_world, 0);
		dJointAttach(joint[rs::JOINT3], _body[BODY], _body[FACE3]);
		dJointSetFixed(joint[rs::JOINT3]);
	}
	else {
		joint[rs::JOINT3] = dJointCreateHinge(_world, 0);
		dJointAttach(joint[rs::JOINT3], _body[BODY], _body[FACE3]);
		this->multiplyQbyV(q, _body_width/2, 0, 0, o);
		dJointSetHingeAnchor(joint[rs::JOINT3], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		this->multiplyQbyV(q, -1, 0, 0, o);
		dJointSetHingeAxis(joint[rs::JOINT3], o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[FACE3], o[0], o[1], o[2]);
	}

	// TODO: build rotated joints
	/*if (_motor[rs::JOINT1].theta != 0) {
		dRFromAxisAndAngle(R_f, -1, 0, 0, _motor[rs::JOINT1].theta);
		this->getRobotBodyOffset(FACE1, p, q, p1, q1);
		this->build_face(FACE1, geom[FACE1], p1, q1);
	}
	if (_motor[rs::JOINT2].theta != 0) {
		dRFromAxisAndAngle(R_f, 0, -1, 0, _motor[rs::JOINT2].theta);
		this->getRobotBodyOffset(FACE2, p, q, p1, q1);
		this->build_face(FACE2, geom[FACE2], p1, q1);
	}
	if (_motor[rs::JOINT3].theta != 0) {
		dRFromAxisAndAngle(R_f, 1, 0, 0, _motor[rs::JOINT3].theta);
		this->getRobotBodyOffset(FACE3, p, q, p1, q1);
		this->build_face(FACE3, geom[FACE3], p1, q1);
	}*/

	// build motors
	for (int i = 0; i < 3; i++) {
		_motor[i].id = dJointCreateAMotor(_world, 0);
		_motor[i].joint = joint[i];
		dJointAttach(_motor[i].id, _body[BODY], _body[FACE1 + i]);
		dJointSetAMotorMode(_motor[i].id, dAMotorUser);
		dJointSetAMotorNumAxes(_motor[i].id, 1);
		dJointSetAMotorAngle(_motor[i].id, 0, 0);
		dJointSetAMotorParam(_motor[i].id, dParamFMax, _motor[i].tau_max);
		dJointSetAMotorParam(_motor[i].id, dParamFudgeFactor, 0.3);
		dJointDisable(_motor[i].id);
	}
	this->multiplyQbyV(q, 1, 0, 0, o);
	dJointSetAMotorAxis(_motor[rs::JOINT1].id, 0, 1, o[0], o[1], o[2]);
	this->multiplyQbyV(q, 0, 1, 0, o);
	dJointSetAMotorAxis(_motor[rs::JOINT2].id, 0, 1, o[0], o[1], o[2]);
	this->multiplyQbyV(q, -1, 0, 0, o);
	dJointSetAMotorAxis(_motor[rs::JOINT3].id, 0, 1, o[0], o[1], o[2]);

	// set damping on all bodies to 0.1
	for (int i = 0; i < NUM_PARTS; i++) dBodySetDamping(_body[i], 0.1, 0.1);

	// delete arrays
	for (int i = 0; i < NUM_PARTS; i++) { delete [] geom[i]; }
	delete [] geom;
	delete [] joint;

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
		_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_motor[id].joint), dJointGetHingeAngleRate(_motor[id].joint)) - _motor[id].offset;

    return _motor[id].theta;
}

void CLinkbotT::init_params(void) {
	_dof = 3;

	// create arrays for linkbots
	_motor.resize(_dof);
	_neighbor.resize(_dof);

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
	_sim = NULL;
	_speed = 2;
	_trace = 1;
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
	for (int i = 0; i < _dof; i++) {
		if (_disabled == i) continue;
		// store current angle
		_motor[i].theta = getAngle(i);
		// set rotation axis
		dVector3 axis;
		dJointGetHingeAxis(_motor[i].joint, axis);
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
	if (_motor[rs::JOINT1].success && _motor[rs::JOINT2].success && _motor[rs::JOINT3].success)
		COND_SIGNAL(&_success_cond);

	// unlock angle and goal
	MUTEX_UNLOCK(&_success_mutex);
	MUTEX_UNLOCK(&_theta_mutex);
	MUTEX_UNLOCK(&_goal_mutex);
}

/**********************************************************
	private functions
 **********************************************************/
void CLinkbotT::build_body(dGeomID *geom, const double *p, const double *q) {
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

void CLinkbotT::build_bridge(Connector *conn) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, _bridge_length, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateBox(_space, _conn_depth, _bridge_length, _conn_height);
	dGeomSetBody(geom[0], conn->body);
}

void CLinkbotT::build_caster(Connector *conn, int custom) {
	// create body
	dGeomID *geom = new dGeomID[4];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 1000, 2*_conn_depth, 1.5*_face_radius, _body_height);
	dMassTranslate(&m, 8*_conn_depth, 0, -_body_height/2);

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, _conn_depth, 1.5*_face_radius, _body_height);
	dGeomSetBody(geom[0], conn->body);

	// default 3ds caster
	if (!custom) {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.0368, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn->body);
		dGeomSetOffsetPosition(geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, 0.003);
		dGeomSetBody(geom[2], conn->body);
		dGeomSetOffsetPosition(geom[2], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 + 0.0001 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn->body);
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 - 0.005 - m.c[2]);
	}
	// custom drawn one for mathematics
	else {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.02, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn->body);
		dGeomSetOffsetPosition(geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, _radius -_face_radius - 0.006 + 0.0032);
		dGeomSetBody(geom[2], conn->body);
		dGeomSetOffsetPosition(geom[2], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 - (_radius -_face_radius - 0.006)/2 + 0.0016 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn->body);
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 + _face_radius - _radius + 0.006 - m.c[2]);
	}

	// center body mass on geoms
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);
}

void CLinkbotT::build_cube(Connector *conn) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 100, _cubic_length, _cubic_length, _cubic_length);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateBox(_space, _cubic_length, _cubic_length, _cubic_length);
	dGeomSetBody(geom[0], conn->body);
}

void CLinkbotT::build_face(int id, dGeomID *geom, const double *p, const double *q) {
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

	// set geometry
	geom[0] = dCreateCylinder(_space, _face_radius, _face_depth);
	dGeomSetBody(geom[0], _body[id]);
	dQuaternion Q1 = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom[0], Q1);
}

void CLinkbotT::build_faceplate(Connector *conn) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, _body_height, _body_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateBox(_space, _conn_depth, _body_height, _body_height);
	dGeomSetBody(geom[0], conn->body);
}

void CLinkbotT::build_gripper(Connector *conn, int face) {
	// create body
	dGeomID *geom = new dGeomID[3];
	int i = (face == 1) ? 1 : -1;

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);

	// set geometry 0
	geom[0] = dCreateBox(_space, _conn_depth, 4*_face_radius, _conn_height/2);
	dGeomSetBody(geom[0], conn->body);
	dGeomSetOffsetPosition(geom[0], -m.c[0], -i*_face_radius - m.c[1], -m.c[2]);

	// set geometry 1
	geom[1] = dCreateBox(_space, 0.062, 0.04, _conn_depth);
	dGeomSetBody(geom[1], conn->body);
	dGeomSetOffsetPosition(geom[1], _conn_depth/2 - 0.062/2 - m.c[0], -i*3*_face_radius + i*0.02 - m.c[1], i*_conn_height/4 - i*_conn_depth/2 - m.c[2]);

	// set geometry 2
	geom[2] = dCreateBox(_space, 0.0344, 0.04, 0.007);
	dGeomSetBody(geom[2], conn->body);
	dGeomSetOffsetPosition(geom[2], _conn_depth/2 - 0.062 + 0.0344/2 - m.c[0], -i*3*_face_radius + i*0.02 - m.c[1], i*_conn_height/4 - i*_conn_depth/2 - i*0.007/2 - m.c[2]);

	// center body mass on geoms
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);
}

void CLinkbotT::build_omnidrive(Connector *conn) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _omni_length, _omni_length, _conn_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateBox(_space, _conn_depth, _omni_length, _omni_length);
	dGeomSetBody(geom[0], conn->body);
}

void CLinkbotT::build_simple(Connector *conn) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// set mass of body
	dMass m;
	dMassSetBox(&m, 170, _conn_depth, 2*_face_radius, _conn_height);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateBox(_space, _conn_depth, 2*_face_radius, _conn_height);
	dGeomSetBody(geom[0], conn->body);
}

void CLinkbotT::build_wheel(Connector *conn, double size) {
	// create body
	dGeomID *geom = new dGeomID[1];

	// store wheel radius
	_wheel_radius = size;

	// set mass of body
	dMass m;
	dMassSetCylinder(&m, 170, 1, 2*_wheel_radius, _wheel_depth);
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(conn->body, &m);

	// set geometry
	geom[0] = dCreateCylinder(_space, _wheel_radius, _wheel_depth);
	dGeomSetBody(geom[0], conn->body);
	dQuaternion Q = {cos(0.785398), 0, sin(0.785398), 0};
	dGeomSetOffsetQuaternion(geom[0], Q);
}

void* CLinkbotT::closeGripperNBThread(void *arg) {
	// cast arg
	LinkbotMove *move = (LinkbotMove *)arg;

	// perform motion
	move->robot->closeGripper();

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

