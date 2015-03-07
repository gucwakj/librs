#include <cstring>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32

#include <iostream>
#include <rsRobots/rgbhashtable.h>
#include <rsSim/Sim>
#include <rsSim/Robot>

using namespace rsSim;

Robot::Robot(void) : rsRobots::Robot(rs::ROBOT) {
	MUTEX_INIT(&_goal_mutex);
	MUTEX_INIT(&_success_mutex);
	COND_INIT(&_success_cond);
	MUTEX_INIT(&_theta_mutex);

	_connected = 1;
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;
	_seed = time(NULL);
}

Robot::~Robot(void) {
	// destroy mutexes
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
	MUTEX_DESTROY(&_theta_mutex);
}

int Robot::getForm(void) {
	return _form;
}

int Robot::getID(void) {
	// get id of robot
	return _id;
}

int Robot::holdJoint(int id) {
	this->setJointSpeed(id, 0);

	// success
	return 0;
}

int Robot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		this->setJointSpeed(static_cast<int>(i), 0);
	}

	// success
	return 0;
}

int Robot::moveJoint(int id, double angle) {
	this->moveJointNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointNB(int id, double angle) {
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

int Robot::moveJointTo(int id, double angle) {
	this->moveJointToNB(id, angle);
	this->moveJointWait(id);

	// success
	return 0;
}

int Robot::moveJointToNB(int id, double angle) {
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

int Robot::moveJointWait(int id) {
	// wait for motion to complete
	MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

int Robot::moveNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointNB(i, angles[i]);
	}

	// success
	return 0;
}

int Robot::moveToNB(double *angles) {
	for (int i = 0; i < _dof; i++) {
		this->moveJointToNB(static_cast<int>(i), angles[i]);
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
		success += _motor[i].success;
	}
	// wait
	while (success != _dof) {
		COND_WAIT(&_success_cond, &_success_mutex);
		success = 0;
		for (int i = 0; i < _dof; i++) { success += _motor[static_cast<int>(i)].success; }
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

int Robot::setJointSpeed(int id, double speed) {
	if (speed > RAD2DEG(_motor[id].omega_max)) {
		fprintf(stderr, "Warning: Setting the speed for joint %d to %.2lf degrees per second is "
			"beyond the hardware limit of %.2lf degrees per second.\n",
			id+1, speed, RAD2DEG(_motor[id].omega_max));
	}
	_motor[id].omega = DEG2RAD(speed);

	// success
	return 0;
}

int Robot::setJointSpeedRatio(int id, double ratio) {
	if ( ratio < 0 || ratio > 1 ) {
		return -1;
	}
	return this->setJointSpeed(id, ratio * RAD2DEG(_motor[(int)id].omega_max));
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

