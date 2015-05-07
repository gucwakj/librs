#include <cstring>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <rs/Macros>
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
	// delete robot from simulation
	if (_sim)
		_sim->deleteRobot(_id);

	// destroy mutexes
	MUTEX_DESTROY(&_goal_mutex);
	MUTEX_DESTROY(&_success_mutex);
	COND_DESTROY(&_success_cond);
	MUTEX_DESTROY(&_theta_mutex);
}

double Robot::getAngle(int id) {
	return _motor[id].theta;
}

int Robot::holdJoint(int id) {
	_motor[id].omega = 0;

	// success
	return 0;
}

int Robot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		_motor[i].omega = 0;
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
	if (_motor[id].omega < -rs::EPSILON) angle = -angle;
	_motor[id].goal += rs::D2R(angle);

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
	_motor[id].goal = rs::D2R(angle);

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
		for (int i = 0; i < _dof; i++) { success += _motor[i].success; }
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

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int Robot::addToSim(dWorldID &world, dSpaceID &space, int id, Sim *sim) {
	_world = world;
	_space = dHashSpaceCreate(space);
	_id = id;
	_sim = sim;

	// success
	return 0;
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

const rs::Pos Robot::getPosition(void) {
	const double *p = dBodyGetPosition(_body[0]);
	return rs::Pos(p[0], p[1], p[2]);
}

const rs::Quat Robot::getQuaternion(void) {
	const double *q = dBodyGetQuaternion(_body[0]);
	return rs::Quat(q[1], q[2], q[3], q[0]);
}

double Robot::getRotation(int body, int i) {
	const double *R = dBodyGetRotation(_body[body]);
	double angles[3] = {0};
    if ( fabs(R[8]-1) < rs::EPSILON ) {         // R_31 == 1; theta = rs::PI/2
        angles[0] = atan2(-R[1], -R[2]);		// psi
        angles[1] = rs::PI/2;					// theta
        angles[2] = 0;							// phi
    }
    else if ( fabs(R[8]+1) < rs::EPSILON ) {    // R_31 == -1; theta = -rs::PI/2
        angles[0] = atan2(R[1], R[2]);			// psi
        angles[1] = -rs::PI/2;					// theta
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
    int stp = (int)( fabs(past_ang) / rs::PI );
    double past_ang_mod = fabs(past_ang) - stp*rs::PI;

    if ( (int)(ang_rate*1000) == 0 ) {
        new_ang = past_ang;
    }
    // positive angular velocity, positive angle
    else if ( ang_rate > 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang < 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + 2*rs::PI); }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + rs::PI);   }
        // cross 0
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + rs::PI);   }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // positive angular velocity, negative angle
    else if ( ang_rate > 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang < 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang + past_ang_mod + rs::PI);   }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - rs::PI);   }
    }
    // negative angular velocity, positive angle
    else if ( ang_rate < 0 && past_ang >= 0 ) {
        // cross 180
        if ( cur_ang > 0 && (stp % 2) ) {   new_ang = past_ang + (cur_ang - past_ang_mod - rs::PI);   }
        // negative
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang - past_ang_mod + rs::PI);   }
        // cross 0
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
        // positive
        else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod);  }
    }
    // negative angular velocity, negative angle
    else if ( ang_rate < 0 && past_ang < 0 ) {
        // cross 180
        if ( cur_ang > 0 && !(stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - 2*rs::PI); }
        // negative
        else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod);  }
        // cross 0
        else if ( cur_ang < 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - rs::PI);   }
        // positive
        else if ( cur_ang > 0 && (stp % 2) ) {  new_ang = past_ang + (cur_ang + past_ang_mod - rs::PI);   }
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
	return sigma*(sqrt(-2.0*log(u1))*cos(2*rs::PI*u2));
}

double Robot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0)
		_seed = _seed + 2147483647;
	return ((double)(_seed) * 4.656612875E-10);
}

