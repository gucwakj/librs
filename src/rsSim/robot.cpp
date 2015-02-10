#include <rsSim/sim.hpp>
#include <rsSim/robot.hpp>

using namespace rsSim;

Robot::Robot(rs::JointID leftWheel, rs::JointID rightWheel) : rsRobots::Robot(rs::ROBOT) {
	_leftWheel = leftWheel;
	_rightWheel = rightWheel;

	MUTEX_INIT(&_active_mutex);
	COND_INIT(&_active_cond);
	MUTEX_INIT(&_goal_mutex);
	MUTEX_INIT(&_motion_mutex);
	COND_INIT(&_motion_cond);
	MUTEX_INIT(&_recording_mutex);
	COND_INIT(&_recording_cond);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);
	MUTEX_INIT(&_theta_mutex);

	_seed = time(NULL);
}

Robot::~Robot(void) {
	// destroy mutexes
	MUTEX_DESTROY(&_active_mutex);
	COND_DESTROY(&_active_cond);
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_motion_mutex);
	COND_DESTROY(&_motion_cond);
	MUTEX_DESTROY(&_recording_mutex);
	COND_DESTROY(&_recording_cond);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
	MUTEX_DESTROY(&_theta_mutex);
}

int Robot::blinkLED(double delay, int num) {
	// store original led color
	double rgb[3] = {_rgb[0], _rgb[1], _rgb[2]};

	// blink num-1 full times
	for (int i = 0; i < num-1; i++) {
		_rgb[0] = 1; _rgb[1] = 1; _rgb[2] = 1;
		this->doze(delay);
		_rgb[0] = rgb[0]; _rgb[1] = rgb[1]; _rgb[2] = rgb[2];
		this->doze(delay);
	}

	// one last off before resetting to original color
	_rgb[0] = 1; _rgb[1] = 1; _rgb[2] = 1;
	this->doze(delay);
	_rgb[0] = rgb[0]; _rgb[1] = rgb[1]; _rgb[2] = rgb[2];

	// success
	return 0;
}

int Robot::connect(char *name, int pause) {
	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

int Robot::delay(double milliseconds) {
	// set ending time
	//double end = g_sim->getClock() + milliseconds/1000;
	double end = _sim->getClock() + milliseconds/1000;

	// while clock hasn't reached ending time
	//while ((end - g_sim->getClock()) >= EPSILON)
	while ((end - _sim->getClock()) >= EPSILON)
		this->doze(50);

	// success
	return 0;
}

int Robot::delaySeconds(double seconds) {
	// delay milliseconds
	this->delay(1000 * seconds);

	// success
	return 0;
}

int Robot::disableRecordDataShift(void) {
	_g_shift_data = 0;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int Robot::disconnect(void) {
	// and we are not connected
	_connected = 0;

	// success
	return 0;
}

int Robot::driveBackward(double angle) {
	this->driveForwardNB(-angle);
	this->moveWait();

	// success
	return 0;
}

int Robot::driveBackwardNB(double angle) {
	this->driveForwardNB(-angle);

	// success
	return 0;
}

int Robot::driveDistance(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));
	this->moveWait();

	// success
	return 0;
}

int Robot::driveDistanceNB(double distance, double radius) {
	this->driveForwardNB(RAD2DEG(distance/radius));

	// success
	return 0;
}

int Robot::driveForever(void) {
	this->driveForeverNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::driveForeverNB(void) {
	this->moveJointForeverNB(_leftWheel);
	this->moveJointForeverNB(_rightWheel);

	// success
	return 0;
}

int Robot::driveForward(double angle) {
	this->driveForwardNB(angle);
	this->moveWait();

	// success
	return 0;
}

int Robot::driveForwardNB(double angle) {
	this->moveJointNB(_leftWheel, angle);
	this->moveJointNB(_rightWheel, angle);

	// success
	return 0;
}

int Robot::driveTime(double seconds) {
	// move joint
	this->driveForeverNB();

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoints();

	// success
	return 0;
}

int Robot::driveTimeNB(double seconds) {
	// set up threading
	THREAD_T moving;
	Recording *rec = new Recording;
	rec->robot = this;
	rec->msecs = 1000*seconds;

	// set joint movements
	this->driveForeverNB();

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::driveTimeNBThread, (void *)rec);

	// success
	return 0;
}

int Robot::drivexy(double x, double y, double radius, double trackwidth) {
	this->drivexyNB(x, y, radius, trackwidth);
	this->drivexyWait();

	// success
	return 0;
}

int Robot::drivexyNB(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// move to new global coordinates
	return this->drivexyToNB(x + x0, y + y0, radius, trackwidth);
}

int Robot::drivexyTo(double x, double y, double radius, double trackwidth) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// if movement is too small, just call it good
	if (fabs(x-x0) < 0.1 && fabs(y-y0) < 0.1) {
		return 1;
	}

	// get current rotation
	double r0 = this->getRotation(0, 2);

	// compute rotation matrix for body frame
	dMatrix3 R;
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// get angle to turn in body coordinates (transform of R)
	double angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));

	// turn toward new postition until pointing correctly
	while (fabs(angle) > 0.01) {
		// turn in shortest path
		if (angle > 0.01)
			this->turnRight(RAD2DEG(angle), radius, trackwidth);
		else if (angle < -0.01)
			this->turnLeft(RAD2DEG(-angle), radius, trackwidth);

		// calculate new rotation from error
		this->getxy(x0, y0);
		r0 = this->getRotation(0, 2);
		dRSetIdentity(R);
		dRFromAxisAndAngle(R, 0, 0, 1, r0);
		angle = atan2(R[0]*(x-x0) + R[4]*(y-y0), R[1]*(x-x0) + R[5]*(y-y0));
	}

	// move along length of line
	this->getxy(x0, y0);
	this->driveDistance(sqrt(x*x - 2*x*x0 + x0*x0 + y*y - 2*y*y0 + y0*y0), radius);

	// success
	return 0;
}

int Robot::drivexyToNB(double x, double y, double radius, double trackwidth) {
	// create thread
	THREAD_T moving;

	// store args
	RobotMove *move = new RobotMove;
	move->robot = this;
	move->x = x;
	move->y = y;
	move->radius = radius;
	move->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&moving, drivexyToThread, (void *)move);

	// success
	return 0;
}

int Robot::drivexyToFunc(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// drivexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = func(x);
		this->drivexyTo(x, y, radius, trackwidth);
	}

	// success
	return 0;
}

int Robot::drivexyToFuncNB(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// create thread
	THREAD_T moving;

	// store args
	RobotMove *move = new RobotMove;
	move->robot = this;
	move->x = x0;
	move->y = xf;
	move->i = n;
	move->func = func;
	move->radius = radius;
	move->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&moving, drivexyToFuncThread, (void *)move);

	// success
	return 0;
}

int Robot::drivexyToFuncSmooth(double x0, double xf, int n, double (*func)(double x), double radius, double trackwidth) {
	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// save current linear speed of robot
	_speed = _motor[_leftWheel].omega*radius;

	// drive to starting location
	this->drivexyTo(x0, func(x0), radius, trackwidth);

	// drivexy to sequence of (x,y) values
	for (int i = 0; i < n; i+=3) {
		double x1 = x0 + i*step;
		double x2 = x0 + (i+1)*step;
		double x3 = x0 + (i+2)*step;
		this->drivexyToSmooth(x1, func(x1), x2, func(x2), x3, func(x3), radius, trackwidth);
	}

	// success
	return 0;
}

int Robot::drivexyToPoly(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// init variables
	double *coeff;
	int *power, order = 0;
	char str[5];
	char input[100];
	std::strcpy(input, poly);

	// parse 'fcn' into usable format
	char *var = std::strchr(input, '^');
	if (var != NULL) {
		order = atoi(++var);
		coeff = new double[order+1]();
		power = new int[order+1]();
		for (int i = 0; i < order+1; i++) {
			coeff[i] = 1;
			power[i] = order-i;
		}
		for (int i = 0; i < order; i++) {
			sprintf(str, "^%d", power[i]);
			var = std::strstr(input, str);
			if (var != NULL) {
				if (var[-2] == '*')
					coeff[i] = atof(var-=3);
				else if (var[-2] == ' ' || var[-2] == '=')
					coeff[i] = 1;
				else
					coeff[i] = atof(var-=2);
			}
			else {
				coeff[i] = 0;
			}
		}
		var = std::strrchr(input, 'x');
		var = std::strpbrk(input, "+-");
		if (var != NULL) {
			if (var[1] == ' ')
				var[1] = var[0];
			coeff[order] = atof(++var);
		}
		else {
			coeff[order] = 0;
		}
	}
	else {
		order = 1;
		coeff = new double[order+1];
		power = new int[order+1];
		power[0] = 1;
		var = std::strchr(input, 'x');
		if (var != NULL) {
			if (var[-1] == '*')
				coeff[0] = atof(var-=2);
			else
				coeff[0] = atof(--var);
		}
		var = std::strpbrk(input, "+-");
		if (var != NULL) {
			if (var[1] == ' ')
				var[1] = var[0];
			coeff[1] = atof(++var);
			power[1] = 0;
		}
	}

	// number of steps necessary
	double step = (xf-x0)/(n-1);

	// drivexy to sequence of (x,y) values
	for (int i = 0; i < n; i++) {
		double x = x0 + i*step;
		double y = 0;
		for (int j = 0; j <= order; j++) {
			y += coeff[j]*pow(x, power[j]);
		}
		this->drivexyTo(x, y, radius, 0);
	}

	return 0;
}

int Robot::drivexyToPolyNB(double x0, double xf, int n, char *poly, double radius, double trackwidth) {
	// create thread
	THREAD_T moving;

	// store args
	RobotMove *move = new RobotMove;
	move->robot = this;
	move->x = x0;
	move->y = xf;
	move->i = n;
	move->expr = poly;
	move->radius = radius;
	move->trackwidth = trackwidth;

	// motion in progress
	_motion = true;

	// start thread
	THREAD_CREATE(&moving, drivexyToPolyThread, (void *)move);

	// success
	return 0;
}

int Robot::drivexyToSmooth(double x1, double y1, double x2, double y2, double x3, double y3, double radius, double trackwidth) {
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
	this->setJointSpeed(_leftWheel, RAD2DEG(s1/theta/rho/radius*_speed));
	this->setJointSpeed(_rightWheel, RAD2DEG(s2/theta/rho/radius*_speed));
	this->moveJointNB(_leftWheel, RAD2DEG(s1/radius));
	this->moveJointNB(_rightWheel, RAD2DEG(s2/radius));
	this->delay(theta*rho/_speed*1000);

	// success
	return 0;
}

int Robot::drivexyWait(void) {
	// wait for motion to complete
	MUTEX_LOCK(&_motion_mutex);
	while (_motion) {
		COND_WAIT(&_motion_cond, &_motion_mutex);
	}
	MUTEX_UNLOCK(&_motion_mutex);

	// success
	return 0;
}

int Robot::enableRecordDataShift(void) {
	_g_shift_data = 1;
	_g_shift_data_en = 1;

	// success
	return 0;
}

int Robot::getAccelerometerData(double &accel_x, double &accel_y, double &accel_z) {
	// output current accel data
	accel_x = _accel[0];
	accel_y = _accel[1];
	accel_z = _accel[2];

	// success
	return 0;
}

int Robot::getBatteryVoltage(double &voltage) {
	voltage = 100;

	// success
	return 0;
}

int Robot::getDistance(double &distance, double radius) {
	double angle;
	this->getJointAngle(rs::JOINT1, angle, 2);
	distance = DEG2RAD(angle) * radius;

	// success
	return 0;
}

int Robot::getFormFactor(int &formFactor) {
	formFactor = _form;

	// success
	return 0;
}

int Robot::getID(void) {
	// get id of robot
	return _id;
}

int Robot::getJointAngle(rs::JointID id, double &angle, int numReadings) {
	//initialize variables
	double d;
	angle = 0;

	// get joint angle numReadings times
	for (int i = 0; i < numReadings; i++) {
		if(this->getJointAngleInstant(id, d)) {
			return -1;
		}
		angle += d;
	}

	// store average angle
	angle = angle/numReadings;

	// success
	return 0;
}

int Robot::getJointAngleInstant(rs::JointID id, double &angle) {
	angle = RAD2DEG(_motor[id].theta);

	// success
	return 0;
}

int Robot::getJointMaxSpeed(rs::JointID id, double &maxSpeed) {
	maxSpeed = RAD2DEG(_motor[id].omega_max);

	// success
	return 0;
}

int Robot::getJointSafetyAngle(double &angle) {
	angle = _motor[rs::JOINT1].safety_angle;

	// success
	return 0;
}

int Robot::getJointSafetyAngleTimeout(double &seconds) {
	seconds = _motor[rs::JOINT1].safety_timeout;

	// success
	return 0;
}

int Robot::getJointSpeed(rs::JointID id, double &speed) {
	speed = RAD2DEG(_motor[id].omega);

	// success
	return 0;
}

int Robot::getJointSpeedRatio(rs::JointID id, double &ratio) {
	ratio = _motor[id].omega/_motor[id].omega_max;

	// success
	return 0;
}

int Robot::getLEDColorName(char color[]) {
	rgbHashTable *rgbTable = HT_Create();
	int getRGB[3] = {(int)(255*_rgb[0]), (int)(255*_rgb[1]), (int)(255*_rgb[2])};
	int retval = HT_GetKey(rgbTable, getRGB, color);
	HT_Destroy(rgbTable);

	// success
	return retval;
}

int Robot::getLEDColorRGB(int &r, int &g, int &b) {
	r = (int)(255*_rgb[0]);
	g = (int)(255*_rgb[1]);
	b = (int)(255*_rgb[2]);

	// success
	return 0;
}

int Robot::getxy(double &x, double &y) {
	// return x and y positions
	x = this->convert(this->getCenter(0), 0);
	y = this->convert(this->getCenter(1), 0);

	// success
	return 0;
}

int Robot::holdJoint(rs::JointID id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int Robot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		this->setJointSpeed(static_cast<rs::JointID>(i), 0);
	}

	// success
	return 0;
}

int Robot::holdJointsAtExit(void) {
	// set joint speeds to zero
	this->holdJoints();

	// hold joints still
	this->moveForeverNB();

	// success
	return 0;
}

int Robot::isConnected(void) {
	// return connected status
	return _connected;
}

int Robot::isMoving(void) {
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].state == POSITIVE || _motor[i].state == NEGATIVE) {
			return 1;
		}
	}

	// success
	return 0;
}

int Robot::isNotMoving(void) {
	// oppositve of ismoving
	return !(this->isMoving());
}

int Robot::moveForeverNB(void) {
	// set joint movements
	for (int i = 0; i < _dof; i++) {
		this->moveJointForeverNB(static_cast<rs::JointID>(i));
	}

	// success
	return 0;
}

int Robot::moveJoint(rs::JointID id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointNB(rs::JointID id, double angle) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	if (_motor[id].omega < -EPSILON) angle = -angle;
	_motor[id].goal += DEG2RAD(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[0]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int Robot::moveJointByPowerNB(rs::JointID id, int power) {
	_motor[id].omega = (power/100.0)*_motor[id].omega_max;

	// success
	return 0;
}

int Robot::moveJointForeverNB(rs::JointID id) {
	// lock mutexes
	MUTEX_LOCK(&_motor[id].success_mutex);
	// enable motor
	dJointEnable(_motor[id].id);
	// set motor angle to current angle
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	// set mode
	_motor[id].mode = CONTINUOUS;
	// drive in proper direction
	if ( _motor[id].omega > EPSILON )
		_motor[id].state = POSITIVE;
	else if ( _motor[id].omega < EPSILON )
		_motor[id].state = NEGATIVE;
	else
		_motor[id].state = HOLD;
	// successfully at 'goal'
	_motor[id].success = true;
	// enable bodies for collisions
    dBodyEnable(_body[0]);
	// unlock mutexes
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int Robot::moveJointTime(rs::JointID id, double seconds) {
	// move joint
	this->moveJointForeverNB(id);

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoint(id);

	// success
	return 0;
}

int Robot::moveJointTimeNB(rs::JointID id, double seconds) {
	// set up threading
	THREAD_T moving;

	// recording arguments
	Recording *rec = new Recording;
	rec->robot = this;
	rec->msecs = 1000*seconds;
	rec->id = id;

	// set joint movements
	this->moveJointForeverNB(id);

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::moveJointTimeNBThread, (void *)rec);

	// success
	return 0;
}

int Robot::moveJointTo(rs::JointID id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointToNB(rs::JointID id, double angle) {
	// lock goal
	MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	_motor[id].goal = DEG2RAD(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[0]);
	MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int Robot::moveJointToByTrackPos(rs::JointID id, double angle) {
	this->moveJointToByTrackPosNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointToByTrackPosNB(rs::JointID id, double angle) {
	this->moveJointToNB(id, angle);

	// success
	return 0;
}

int Robot::moveJointWait(rs::JointID id) {
	// wait for motion to complete
	MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int Robot::moveTime(double seconds) {
	// move joint
	this->moveForeverNB();

	// sleep
	this->doze(seconds*1000);

	// stop joint
	this->holdJoints();

	// success
	return 0;
}

int Robot::moveTimeNB(double seconds) {
	// set up threading
	THREAD_T moving;

	// recording arguments
	Recording *rec = new Recording;
	rec->robot = this;
	rec->msecs = 1000*seconds;

	// set joint movements
	this->moveForeverNB();

	// create thread to wait
	THREAD_CREATE(&moving, (void* (*)(void *))&Robot::moveTimeNBThread, (void *)rec);

	// success
	return 0;
}

int Robot::moveToZero(void) {
	this->moveToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::moveToZeroNB(void) {
	// move joints to zero
	for (int i = 0; i < _dof; i++) {
		this->moveJointToNB(static_cast<rs::JointID>(i), 0);
	}

	// success
	return 0;
}

int Robot::moveWait(void) {
	// lock
	MUTEX_LOCK(&_success_mutex);
	// get number of successes
	int success = 0;
	for (int i = 0; i < _dof; i++) {
		success += _motor[static_cast<rs::JointID>(i)].success;
	}
	// wait
	while (success != _dof) {
		COND_WAIT(&_success_cond, &_success_mutex);
		success = 0;
		for (int i = 0; i < _dof; i++) { success += _motor[static_cast<rs::JointID>(i)].success; }
	}
	// reset motor states
	for (int i = 0; i < _dof; i++) {
		_motor[i].mode = CONTINUOUS;
	}
	// unlock
	MUTEX_UNLOCK(&_success_mutex);

	// success
	return 0;
}

int Robot::recordAngle(rs::JointID id, double time[], double angle[], int num, double seconds, int shiftData) {
	// check if recording already
	if (_motor[id].record) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args
	Recording *rec = new Recording;
	rec->robot = this;
	rec->time = time;
	rec->angle = new double * [1];
	rec->angle[0] = angle;
	rec->id = id;
	rec->num = num;
	rec->msecs = 1000*seconds;

	// lock recording for joint id
	_motor[id].record = true;

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAngleThread, (void *)rec);

	// success
	return 0;
}

int Robot::recordAngleBegin(rs::JointID id, robotRecordData_t &time, robotRecordData_t &angle, double seconds, int shiftData) {
	// check if recording already
	if (_motor[id].record) { return -1; }

	// set up recording thread
	THREAD_T recording;

	// set up recording args
	Recording *rec = new Recording;
	rec->robot = this;
	rec->id = id;
	rec->num = RECORD_ANGLE_ALLOC_SIZE;
	rec->msecs = seconds * 1000;
	time = new double[RECORD_ANGLE_ALLOC_SIZE];
	angle = new double[RECORD_ANGLE_ALLOC_SIZE];
	rec->ptime = &time;
	rec->pangle = new double ** [_dof];
	rec->pangle[rs::JOINT1] = &angle;

	// store pointer to recorded angles locally
	_motor[id].record_angle = &angle;

	// lock recording for joint id
	_motor[id].record = true;

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAngleBeginThread, (void *)rec);

	// success
	return 0;
}

int Robot::recordAngleEnd(rs::JointID id, int &num) {
	// sleep to capture last data point on ending time
	this->doze(150);

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_motor[id].record = false;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_motor[id].record_active) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _motor[id].record_num;

	// success
	return 0;
}

int Robot::recordAnglesEnd(int &num) {
	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	for (int i = 0; i < _dof; i++) {
		_motor[i].record = 0;
	}
	MUTEX_UNLOCK(&_recording_mutex);

	// get number of joints recording
	int rec = 0;
	for (int i = 0; i < _dof; i++) {
		rec += _motor[i].record_active;
	}
	MUTEX_LOCK(&_active_mutex);
	while (rec) {
		COND_WAIT(&_active_cond, &_active_mutex);
		rec = 0;
		for (int i = 0; i < _dof; i++) { rec += _motor[i].record_active; }
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _motor[rs::JOINT1].record_num;

	// success
	return 0;
}

int Robot::recordDistanceBegin(rs::JointID id, robotRecordData_t &time, robotRecordData_t &distance, double radius, double seconds, int shiftData) {
	// record angle of desired joint
	this->recordAngleBegin(id, time, distance, seconds, shiftData);

	// success
	return 0;
}

int Robot::recordDistanceEnd(rs::JointID id, int &num) {
	// end recording of angles
	this->recordAngleEnd(id, num);

	// convert radius to output units
	double radius = this->convert(_radius, 0);

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		(*_motor[id].record_angle)[i] = DEG2RAD((*_motor[id].record_angle)[i]) * radius + _distOffset;
	}

	// success
	return 0;
}

int Robot::recordDistanceOffset(double distance) {
	// get current position
	double x0, y0;
	this->getxy(x0, y0);

	// get current rotation
	dMatrix3 R;
	double r0 = this->getRotation(0, 2);
	dRFromAxisAndAngle(R, 0, 0, 1, r0);

	// calculate y offset from zero in body coordinates
	double y = R[1]*x0 + R[5]*y0;

	// print warning if different from given offset
	if (fabs(y-distance) > 0.01) {
		printf("Warning: Robot position different from the offset specified in recordDistanceOffset(%lf)\n", distance);
	}

	// set offset distance
	_distOffset = distance;

	// success
	return 0;
}

int Robot::recordDistancesEnd(int &num) {
	// end recording of angles
	this->recordAnglesEnd(num);

	// convert radius to output units
	double radius = this->convert(_radius, 0);

	// convert all angles to distances based upon radius
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < _dof; j++) {
			(*_motor[j].record_angle)[i] = DEG2RAD((*_motor[j].record_angle)[i]) * radius + _distOffset;
		}
	}

	// success
	return 0;
}

int Robot::recordWait(void) {
	// lock
	MUTEX_LOCK(&_recording_mutex);
	// get number of joints recording
	int recording = 0;
	for (int i = 0; i < _dof; i++) {
		recording += _motor[i].record;
	}
	// wait
	while (recording) {
		COND_WAIT(&_recording_cond, &_recording_mutex);
	}
	// unlock
	MUTEX_UNLOCK(&_recording_mutex);

	// success
	return 0;
}

int Robot::recordxyBegin(robotRecordData_t &x, robotRecordData_t &y, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < 2; i++) {
		if (_motor[i].record) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args
	Recording *rec = new Recording;
	rec->robot = this;
	rec->num = RECORD_ANGLE_ALLOC_SIZE;
	rec->msecs = seconds * 1000;
	x = new double[RECORD_ANGLE_ALLOC_SIZE];
	y = new double[RECORD_ANGLE_ALLOC_SIZE];
	rec->ptime = &x;
	rec->pangle = new double ** [1];
	rec->pangle[rs::JOINT1] = &y;

	// store pointer to recorded angles locally
	_motor[rs::JOINT1].record_angle = &x;
	_motor[rs::JOINT2].record_angle = &y;

	// lock recording for joint id
	_motor[rs::JOINT1].record = true;
	_motor[rs::JOINT2].record = true;

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordxyBeginThread, (void *)rec);

	// success
	return 0;
}

int Robot::recordxyEnd(int &num) {
	// sleep to capture last data point on ending time
	this->doze(150);

	// turn off recording
	MUTEX_LOCK(&_recording_mutex);
	_motor[rs::JOINT1].record = false;
	_motor[rs::JOINT2].record = false;
	MUTEX_UNLOCK(&_recording_mutex);

	// wait for last recording point to finish
	MUTEX_LOCK(&_active_mutex);
	while (_motor[rs::JOINT1].record_active && _motor[rs::JOINT2].record_active) {
		COND_WAIT(&_active_cond, &_active_mutex);
	}
	MUTEX_UNLOCK(&_active_mutex);

	// report number of data points recorded
	num = _motor[rs::JOINT1].record_num;

	// convert recorded values into in/cm
	double m2x = (1, 0);
	for (int i = 0; i < num; i++) {
		(*_motor[rs::JOINT1].record_angle)[i] *= m2x;
		(*_motor[rs::JOINT2].record_angle)[i] *= m2x;
	}

	// success
	return 0;
}

int Robot::relaxJoint(rs::JointID id) {
	dJointDisable(_motor[id].id);

	// success
	return 0;
}

int Robot::relaxJoints(void) {
	for (int i = 0; i < _dof; i++) {
		this->relaxJoint(static_cast<rs::JointID>(i));
	}

	// success
	return 0;
}

int Robot::resetToZero(void) {
	this->resetToZeroNB();
	this->moveWait();

	// success
	return 0;
}

int Robot::resetToZeroNB(void) {
	// reset absolute counter to 0 -> 2M_PI
	MUTEX_LOCK(&_theta_mutex);
	for (int i = 0; i < _dof; i++) {
		int rev = (int)(_motor[i].theta/2/M_PI);
		if (rev) {
			_motor[i].theta -= 2*rev*M_PI;
			_motor[i].goal -= 2*rev*M_PI;
		}
	}
	MUTEX_UNLOCK(&_theta_mutex);

	// move to zero position
	this->moveToZeroNB();

	// success
	return 0;
}

int Robot::setBuzzerFrequency(int frequency, double time) {
	printf("::setBuzzerFrequency not implemented.\n");

	// success
	return 0;
}

int Robot::setBuzzerFrequencyOff(void) {
	printf("::setBuzzerFrequencyOff not implemented.\n");

	// success
	return 0;
}

int Robot::setBuzzerFrequencyOn(int frequency) {
	printf("::setBuzzerFrequencyOn not implemented.\n");

	// success
	return 0;
}

int Robot::setLEDColor(char *color) {
	int getRGB[3] = {0};
	rgbHashTable *rgbTable = HT_Create();
	int htRetval = HT_Get(rgbTable, color, getRGB);
	HT_Destroy(rgbTable);

	if (htRetval) {
		_rgb[0] = getRGB[0]/255.0;
		_rgb[1] = getRGB[1]/255.0;
		_rgb[2] = getRGB[2]/255.0;

		// success
		return 0;
	}
	else {
		return htRetval;
	}
}

int Robot::setLEDColorRGB(int r, int g, int b) {
	_rgb[0] = r/255.0;
	_rgb[1] = g/255.0;
	_rgb[2] = b/255.0;

	// success
	return 0;
}

int Robot::setJointSafetyAngle(double angle) {
	for (int i = 0; i < _dof; i++) {
		_motor[i].safety_angle = angle;
	}

	// success
	return 0;
}

int Robot::setJointSafetyAngleTimeout(double seconds) {
	for (int i = 0; i < _dof; i++) {
		_motor[i].safety_timeout = seconds;
	}

	// success
	return 0;
}

int Robot::setJointSpeed(rs::JointID id, double speed) {
	if (speed > RAD2DEG(_motor[id].omega_max)) {
		fprintf(stderr, "Warning: Setting the speed for joint %d to %.2lf degrees per second is "
			"beyond the hardware limit of %.2lf degrees per second.\n",
			id+1, speed, RAD2DEG(_motor[id].omega_max));
	}
	_motor[id].omega = DEG2RAD(speed);

	// success
	return 0;
}

int Robot::setJointSpeedRatio(rs::JointID id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * RAD2DEG(_motor[(int)id].omega_max));
}

int Robot::setSpeed(double speed, double radius) {
	if (RAD2DEG(speed/radius) > RAD2DEG(_motor[rs::JOINT1].omega_max)) {
		fprintf(stderr, "Warning: Speed %.2lf corresponds to joint speeds of %.2lf degrees/second.\n",
			speed, RAD2DEG(speed/radius));
	}
	_speed = speed;
	this->setJointSpeed(_leftWheel, RAD2DEG(speed/radius));
	this->setJointSpeed(_rightWheel, RAD2DEG(speed/radius));

	// success
	return 0;
}

int Robot::systemTime(double &time) {
	// get time
	//time = g_sim->getClock();
	time = _sim->getClock();

	// success
	return 0;
}

int Robot::traceOff(void) {
	_trace = 0;

	// success
	return 0;
}

int Robot::traceOn(void) {
	_trace = 1;

	// success
	return 0;
}

int Robot::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int Robot::turnLeftNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = this->convert(_trackwidth, 0);

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move left joint backward
	this->moveJointNB(_leftWheel, -angle);
	// move right joint forward
	this->moveJointNB(_rightWheel, angle);

	// success
	return 0;
}

int Robot::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	this->moveWait();

	// success
	return 0;
}

int Robot::turnRightNB(double angle, double radius, double trackwidth) {
	// use internally calculated track width
	double width = this->convert(_trackwidth, 0);

	// calculate joint angle from global turn angle
	angle = (angle*width)/(2*radius);

	// move left joint forward
	this->moveJointNB(_leftWheel, angle);
	// move right joint backward
	this->moveJointNB(_rightWheel, -angle);

	// success
	return 0;
}

/**********************************************************
	protected functions for variable DOF
 **********************************************************/
int Robot::moveNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointNB(static_cast<rs::JointID>(i), angles[i]);
	}

	// success
	return 0;
}

int Robot::moveToNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointToNB(static_cast<rs::JointID>(i), angles[i]);
	}

	// success
	return 0;
}

int Robot::recordAngles(double *time, double **angle, int num, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args
	Recording *rec = new Recording;
	rec->robot = this;
	rec->time = time;
	rec->angle = new double * [_dof];
	rec->angle = angle;
	rec->num = num;
	rec->msecs = 1000*seconds;

	// lock recording for joints
	for (int i = 0; i < _dof; i++) {
		rec->angle[i] = angle[i];
		_motor[i].record = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAnglesThread, (void *)rec);

	// success
	return 0;
}

int Robot::recordAnglesBegin(robotRecordData_t &time, robotRecordData_t *&angle, double seconds, int shiftData) {
	// check if recording already
	for (int i = 0; i < _dof; i++) {
		if (_motor[i].record) { return -1; }
	}

	// set up recording thread
	THREAD_T recording;

	// set up recording args
	Recording *rec = new Recording;
	rec->robot = this;
	rec->num = RECORD_ANGLE_ALLOC_SIZE;
	rec->msecs = seconds * 1000;
	time = new double[RECORD_ANGLE_ALLOC_SIZE];
	for (int i = 0; i < _dof; i++) {
		angle[i] = new double[RECORD_ANGLE_ALLOC_SIZE];
	}
	rec->ptime = &time;
	rec->pangle = new double ** [_dof];
	for (int i = 0; i < _dof; i++) {
		rec->pangle[i] = &angle[i];
	}

	// store pointer to recorded angles locally
	for (int i = 0; i < _dof; i++) {
		_motor[i].record_angle = &angle[i];
	}

	// lock recording for joint id
	for (int i = 0; i < _dof; i++) {
		_motor[i].record = true;
	}

	// set shift data
	_shift_data = shiftData;

	// create thread
	THREAD_CREATE(&recording, (void* (*)(void *))&Robot::recordAnglesBeginThread, (void *)rec);

	// success
	return 0;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int Robot::addToSim(dWorldID &world, dSpaceID &space, int id, int pos, Sim *sim) {
	_world = world;
	_space = dHashSpaceCreate(space);
	_id = id;
	_pos = pos;
	_sim = sim;

	// success
	return 0;
}

double Robot::convert(double value, int tometer) {
	double tmp = 0;

	/*if (tometer)
		tmp = ((g_sim->getUnits()) ? value/39.370 : value/100);
	else
		tmp = ((g_sim->getUnits()) ? value*39.370 : value*100);*/
	/*if (tometer)
		tmp = ((_sim->getUnits()) ? value/39.370 : value/100);
	else
		tmp = ((_sim->getUnits()) ? value*39.370 : value*100);*/

	return tmp;
}

int Robot::doze(double ms) {
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms*1000);
#endif
	// success
	return 0;
}

int Robot::fixBodyToGround(dBodyID cbody) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, 0, cbody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

dBodyID Robot::getBodyID(int id) {
	return _body[id];
}

BodyList& Robot::getBodyList(void) {
	return _body;
}

double Robot::getCenter(int i) {
	const double *pos = dBodyGetPosition(_body[0]);
	const double *R = dBodyGetRotation(_body[0]);
	double p[3] = {	R[0]*_center[0] + R[1]*_center[1] + R[2]*_center[2],
					R[4]*_center[0] + R[5]*_center[1] + R[6]*_center[2],
					R[8]*_center[0] + R[9]*_center[1] + R[10]*_center[2]};
	return pos[i] + p[i];
}

const double* Robot::getPosition(void) {
	return dBodyGetPosition(_body[0]);
}

const double* Robot::getQuaternion(void) {
	const double *quat = dBodyGetQuaternion(_body[0]);
	double *q = new double[4];
	q[0] = quat[1];
	q[1] = quat[2];
	q[2] = quat[3];
	q[3] = quat[0];
	return q;
}

double Robot::getRotation(int body, int i) {
	const double *R = dBodyGetRotation(_body[body]);
	double angles[3] = {0};
    if ( fabs(R[8]-1) < DBL_EPSILON ) {         // R_31 == 1; theta = M_PI/2
        angles[0] = atan2(-R[1], -R[2]);		// psi
        angles[1] = M_PI/2;						// theta
        angles[2] = 0;							// phi
    }
    else if ( fabs(R[8]+1) < DBL_EPSILON ) {    // R_31 == -1; theta = -M_PI/2
        angles[0] = atan2(R[1], R[2]);			// psi
        angles[1] = -M_PI/2;					// theta
        angles[2] = 0;							// phi
    }
    else {
        angles[1] = asin(R[8]);
        angles[0] = atan2(R[9]/cos(angles[0]), R[10]/cos(angles[0]));
        angles[2] = atan2(R[4]/cos(angles[0]), R[0]/cos(angles[0]));
    }
	return angles[i];
}

double Robot::mod_angle(double past_ang, double cur_ang, double ang_rate) {
    double new_ang = 0;
    int stp = (int)( fabs(past_ang) / M_PI );
    double past_ang_mod = fabs(past_ang) - stp*M_PI;

    if ( (int)(ang_rate*1000) == 0 ) {
        new_ang = past_ang;
    }
    // positive angular velocity, positive angle
    else if ( ang_rate > 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang < 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + 2*M_PI); }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // cross 0
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // positive angular velocity, negative angle
    else if ( ang_rate > 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang < 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang + past_ang_mod + M_PI);   }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
    }
    // negative angular velocity, positive angle
    else if ( ang_rate < 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang > 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang - past_ang_mod - M_PI);   }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + M_PI);   }
        // cross 0
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // negative angular velocity, negative angle
    else if ( ang_rate < 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang > 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - 2*M_PI); }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - M_PI);   }
    }

    return new_ang;
}

int Robot::noisy(double *a, int length, double sigma) {
	// initialize variables
	double *rand = new double[length];
	double sum = 0;

	if (length == 1)
		a[0] += this->normal(sigma);
	else {
		// compute magnitude of randomized vector
		for (int i = 0; i < length; i++) {
			rand[i] = this->normal(sigma);
			sum += (a[i] + rand[i]) * (a[i] + rand[i]);
		}
		double mag = sqrt(sum);

		// normalize vector
		for (int i = 0; i < length; i++) {
			a[i] = (a[i] + rand[i])/mag;
		}
	}

	// clean up array
	delete [] rand;

	// success
	return 0;
}

void* Robot::simPreCollisionThreadEntry(void *arg) {
	Robot *p = (Robot *)arg;
	p->simPreCollisionThread();
	return arg;
}

void* Robot::simPostCollisionThreadEntry(void *arg) {
	Robot *p = (Robot *)arg;
	p->simPostCollisionThread();
	return arg;
}

/**********************************************************
	private functions
 **********************************************************/
bool Robot::is_shift_enabled(void) {
	if(_shift_data && !_g_shift_data_en)
		return 1;
	else if (_g_shift_data_en && _g_shift_data)
		return 1;
	else
		return 0;
}

double Robot::normal(double sigma) {
	// compute pair of random uniform data
	double u1 = this->uniform();
	double u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*M_PI*u2));
}

double Robot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
}

void* Robot::driveTimeNBThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rec->robot);
	// sleep
	robot->doze(rec->msecs);
	// hold all robot motion
	robot->holdJoints();

	// cleanup
	delete rec;

	// success
	return NULL;
}

void* Robot::drivexyToThread(void *arg) {
	// cast arg
	RobotMove *move = (RobotMove *)arg;

	// perform motion
	move->robot->drivexyTo(move->x, move->y, move->radius, move->trackwidth);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

void* Robot::drivexyToFuncThread(void *arg) {
	// cast arg
	RobotMove *move = (RobotMove *)arg;

	// perform motion
	move->robot->drivexyToFunc(move->x, move->y, move->i, move->func, move->radius, move->trackwidth);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

void* Robot::drivexyToPolyThread(void *arg) {
	// cast arg
	RobotMove *move = (RobotMove *)arg;

	// perform motion
	move->robot->drivexyToPoly(move->x, move->y, move->i, move->expr, move->radius, move->trackwidth);

	// signal successful completion
	COND_ACTION(&move->robot->_motion_cond, &move->robot->_motion_mutex, move->robot->_motion = false);

	// cleanup
	delete move;

	// success
	return NULL;
}

void* Robot::moveJointTimeNBThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rec->robot);
	// sleep
	robot->doze(rec->msecs);
	// hold all robot motion
	robot->holdJoint(rec->id);

	// cleanup
	delete rec;

	// success
	return NULL;
}

void* Robot::moveTimeNBThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// get robot
	Robot *robot = dynamic_cast<Robot *>(rec->robot);
	// sleep
	robot->doze(rec->msecs);
	// hold all robot motion
	robot->holdJoints();

	// cleanup
	delete rec;

	// success
	return NULL;
}

void* Robot::recordAngleThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// create initial time points
	double start_time = 0;
	//int time = (int)(g_sim->getClock()*1000);
	int time = (int)(rec->robot->_sim->getClock()*1000);

	// is robot moving
	int *moving = new int[rec->num];

	// get 'num' data points
	for (int i = 0; i < rec->num; i++) {
		// store time of data point
		//rec->time[i] = g_sim->getClock()*1000;
		rec->time[i] = rec->robot->_sim->getClock()*1000;
		if (i == 0) { start_time = rec->time[i]; }
		rec->time[i] = (rec->time[i] - start_time) / 1000;

		// store joint angle
		rec->angle[0][i] = RAD2DEG(rec->robot->_motor[rec->id].theta);

		// check if joint is moving
		moving[i] = (int)(dJointGetAMotorParam(rec->robot->_motor[rec->id].id, dParamVel)*1000);

		// increment time step
		time += rec->msecs;

		// pause until next step
		//if ( (int)(g_sim->getClock()*1000) < time )
		if ( (int)(rec->robot->_sim->getClock()*1000) < time )
			rec->robot->doze(time - (int)(rec->robot->_sim->getClock()*1000));
			//rec->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rec->robot->is_shift_enabled()) {
		for (int i = 0; i < rec->num; i++) {
			if( moving[i] ) {
				shiftTime = rec->time[i];
				shiftTimeIndex = i;
				break;
			}
		}
		for (int i = 0; i < rec->num; i++) {
			if (i < shiftTimeIndex) {
				rec->time[i] = 0;
				rec->angle[i] = rec->angle[shiftTimeIndex];
			}
			else {
				rec->time[i] = rec->time[i] - shiftTime;
			}
		}
	}

	// signal completion of recording
	COND_ACTION(&rec->robot->_recording_cond, &rec->robot->_recording_mutex, rec->robot->_motor[rec->id].record = false);

	// cleanup
	delete rec;
	delete moving;

	// success
	return NULL;
}

void* Robot::recordAngleBeginThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// create initial time points
	double start_time = 0;
	//int time = (int)((g_sim->getClock())*1000);
	int time = (int)((rec->robot->_sim->getClock())*1000);

	// is robot moving
	int moving;

	// actively taking a new data point
	MUTEX_LOCK(&rec->robot->_active_mutex);
	rec->robot->_motor[rec->id].record_active = true;
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rec->robot->_motor[rec->id].record; i++) {
		// store locally num of data points taken
		rec->robot->_motor[rec->id].record_num = i;

		// resize array if filled current one
		if (i >= rec->num) {
			rec->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newbuf = new double[rec->num];
			memcpy(newbuf, *(rec->ptime), sizeof(double)*i);
			delete [] *(rec->ptime);
			*(rec->ptime) = newbuf;
			// create larger array for angle
			newbuf = new double[rec->num];
			memcpy(newbuf, *(rec->pangle[0]), sizeof(double)*i);
			delete [] *(rec->pangle[0]);
			*(rec->pangle[0]) = newbuf;
		}

		// store joint angles
		(*(rec->pangle[0]))[i] = RAD2DEG(rec->robot->_motor[rec->id].theta);
		moving = (int)(dJointGetAMotorParam(rec->robot->_motor[rec->id].id, dParamVel)*1000);

		// store time of data point
		//(*rec->ptime)[i] = g_sim->getClock()*1000;
		(*rec->ptime)[i] = rec->robot->_sim->getClock()*1000;
		if (i == 0) { start_time = (*rec->ptime)[i]; }
		(*rec->ptime)[i] = ((*rec->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rec->msecs;

		// pause until next step
		//if ( (int)(g_sim->getClock()*1000) < time )
		if ( (int)(rec->robot->_sim->getClock()*1000) < time )
			rec->robot->doze(time - (int)(rec->robot->_sim->getClock()*1000));
			//rec->robot->doze(time - (int)(g_sim->getClock()*1000));

		// wait until movement to start recording
		if( !moving && rec->robot->is_shift_enabled() ) {
			i--;
		}
	}

	// signal completion of recording
	MUTEX_LOCK(&rec->robot->_active_mutex);
	rec->robot->_motor[rec->id].record_active = false;
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// cleanup
	delete rec;

	// success
	return NULL;
}

void* Robot::recordAnglesThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// create initial time points
    double start_time = 0;
	//int time = (int)(g_sim->getClock()*1000);
	int time = (int)(rec->robot->_sim->getClock()*1000);

	// is robot moving
	int *moving = new int[rec->num];

	// get 'num' data points
    for (int i = 0; i < rec->num; i++) {
		// store time of data point
		//rec->time[i] = g_sim->getClock()*1000;
		rec->time[i] = rec->robot->_sim->getClock()*1000;
        if (i == 0) { start_time = rec->time[i]; }
        rec->time[i] = (rec->time[i] - start_time) / 1000;

		// store joint angles
		for (int j = 0; j < rec->robot->_dof; j++) {
			rec->angle[j][i] = RAD2DEG(rec->robot->_motor[j].theta);
		}

		// check if joints are moving
		moving[i] = 0;
		for (int j = 0; j < rec->robot->_dof; j++) {
			moving[i] += (int)(dJointGetAMotorParam(rec->robot->_motor[j].id, dParamVel)*1000);
		}

		// increment time step
		time += rec->msecs;

		// pause until next step
		//if ( (int)(g_sim->getClock()*1000) < time )
		if ( (int)(rec->robot->_sim->getClock()*1000) < time )
			rec->robot->doze(time - (int)(rec->robot->_sim->getClock()*1000));
			//rec->robot->doze(time - (int)(g_sim->getClock()*1000));
    }

	// shift time to start of movement
	double shiftTime = 0;
	int shiftTimeIndex = 0;
	if(rec->robot->is_shift_enabled()) {
		for (int i = 0; i < rec->num; i++) {
			if (moving[i]) {
				shiftTime = rec->time[i];
				shiftTimeIndex = i;
				break;
			}
		}
		for (int i = 0; i < rec->num; i++) {
			if (i < shiftTimeIndex) {
				rec->time[i] = 0;
				for (int j = 0; j < rec->robot->_dof; j++) {
					rec->angle[j][i] = rec->angle[j][shiftTimeIndex];
				}
			}
			else {
				rec->time[i] = rec->time[i] - shiftTime;
			}
		}
	}

	// signal completion of recording
	MUTEX_LOCK(&rec->robot->_recording_mutex);
    for (int i = 0; i < rec->robot->_dof; i++) {
        rec->robot->_motor[i].record = false;
    }
	COND_SIGNAL(&rec->robot->_recording_cond);
	MUTEX_UNLOCK(&rec->robot->_recording_mutex);

	// cleanup
	delete rec;
	delete moving;

	// success
	return NULL;
}

void* Robot::recordAnglesBeginThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// create initial time points
	double start_time = 0;
	//int time = (int)((g_sim->getClock())*1000);
	int time = (int)((rec->robot->_sim->getClock())*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rec->robot->_active_mutex);
	for (int i = 0; i < rec->robot->_dof; i++) {
		rec->robot->_motor[i].record_active = true;
	}
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rec->robot->_motor[rec->id].record; i++) {
		// store locally num of data points taken
		rec->robot->_motor[rs::JOINT1].record_num = i;

		// resize array if filled current one
		if(i >= rec->num) {
			rec->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newbuf = new double[rec->num];
			memcpy(newbuf, *rec->ptime, sizeof(double)*i);
			delete [] *(rec->ptime);
			*(rec->ptime) = newbuf;
			for (int j = 0; j < rec->robot->_dof; j++) {
				newbuf = new double[rec->num];
				memcpy(newbuf, *(rec->pangle[j]), sizeof(double)*i);
				delete [] *(rec->pangle[j]);
				*(rec->pangle[j]) = newbuf;
			}
		}

		// store joint angles
		for (int j = 0; j < rec->robot->_dof; j++) {
			(*(rec->pangle[j]))[i] = RAD2DEG(rec->robot->_motor[j].theta);
		}

		// store time of data point
		//(*rec->ptime)[i] = g_sim->getClock()*1000;
		(*rec->ptime)[i] = rec->robot->_sim->getClock()*1000;
		if (i == 0) { start_time = (*rec->ptime)[i]; }
		(*rec->ptime)[i] = ((*rec->ptime)[i] - start_time) / 1000;

		// increment time step
		time += rec->msecs;

		// pause until next step
		//if ( (int)(g_sim->getClock()*1000) < time )
		if ( (int)(rec->robot->_sim->getClock()*1000) < time )
			rec->robot->doze(time - (int)(rec->robot->_sim->getClock()*1000));
			//rec->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// signal completion of recording
	MUTEX_LOCK(&rec->robot->_active_mutex);
	for (int i = 0; i < rec->robot->_dof; i++) {
		rec->robot->_motor[i].record_active = false;
	}
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// cleanup
	delete rec;

	// success
	return NULL;
}

void* Robot::recordxyBeginThread(void *arg) {
	// cast argument
	Recording *rec = (Recording *)arg;

	// create initial time points
	//int time = (int)((g_sim->getClock())*1000);
	int time = (int)((rec->robot->_sim->getClock())*1000);

	// actively taking a new data point
	MUTEX_LOCK(&rec->robot->_active_mutex);
	rec->robot->_motor[rs::JOINT1].record_active = true;
	rec->robot->_motor[rs::JOINT2].record_active = true;
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// loop until recording is no longer needed
	for (int i = 0; rec->robot->_motor[rs::JOINT1].record; i++) {
		// store locally num of data points taken
		rec->robot->_motor[rs::JOINT1].record_num = i;

		// resize array if filled current one
		if (i >= rec->num) {
			rec->num += RECORD_ANGLE_ALLOC_SIZE;
			// create larger array for time
			double *newbuf = new double[rec->num];
			memcpy(newbuf, *rec->ptime, sizeof(double)*i);
			delete *(rec->ptime);
			*(rec->ptime) = newbuf;
			// create larger array for angle
			newbuf = new double[rec->num];
			memcpy(newbuf, *(rec->pangle), sizeof(double)*i);
			delete (*(rec->pangle));
			*(rec->pangle[0]) = newbuf;
		}

		// store positions
		if (rec->robot->_trace) {
			(*(rec->ptime))[i] = rec->robot->getCenter(0);
			(*(rec->pangle[0]))[i] = rec->robot->getCenter(1);
		}
		else {
			i--;
		}

		// increment time step
		time += rec->msecs;

		// pause until next step
		//if ( (int)(g_sim->getClock()*1000) < time )
		if ( (int)(rec->robot->_sim->getClock()*1000) < time )
			rec->robot->doze(time - (int)(rec->robot->_sim->getClock()*1000));
			//rec->robot->doze(time - (int)(g_sim->getClock()*1000));
	}

	// signal completion of recording
	MUTEX_LOCK(&rec->robot->_active_mutex);
	rec->robot->_motor[rs::JOINT1].record_active = false;
	rec->robot->_motor[rs::JOINT2].record_active = false;
	COND_SIGNAL(&rec->robot->_active_cond);
	MUTEX_UNLOCK(&rec->robot->_active_mutex);

	// cleanup
	delete rec;

	// success
	return NULL;
}

