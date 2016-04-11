#include <cstring>
#include <ctime>

#include <rs/Macros>
#include <rsSim/Sim>
#include <rsSim/Robot>

using namespace rsSim;

Robot::Robot(void) : rsRobots::Robot(rs::Robot) {
	// protected variables
	_connected = false;
	_sim = NULL;
	_success = false;
	_accel[0] = 0;
	_accel[1] = 0;
	_accel[2] = 0;
	_trackwidth = 0;

	// private variables
	_seed = time(NULL);

#ifdef RS_RESEARCH
	_next_goal = 0;		// research: next cpg values to hit
#endif

	// threading
	RS_COND_INIT(&_success_cond);
	RS_MUTEX_INIT(&_goal_mutex);
	RS_MUTEX_INIT(&_success_mutex);
	RS_MUTEX_INIT(&_theta_mutex);
}

Robot::~Robot(void) {
	// delete robot from simulation
	if (_sim) _sim->deleteRobot(this->getID());

	// destroy mutexes
	RS_MUTEX_DESTROY(&_goal_mutex);
	RS_MUTEX_DESTROY(&_success_mutex);
	RS_MUTEX_DESTROY(&_theta_mutex);
	RS_COND_DESTROY(&_success_cond);
}

/**********************************************************
	public functions
 **********************************************************/
double Robot::getAngle(int id) {
	return _motor[id].theta;
}

int Robot::holdJoint(int id) {
	_motor[id].mode = CONTINUOUS;
	_motor[id].state = HOLD;
	_motor[id].omega = 0;

	// success
	return 0;
}

int Robot::holdJoints(void) {
	// set joints to zero speed
	for (int i = 0; i < _dof; i++) {
		this->holdJoint(i);
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
	RS_MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	if (_motor[id].omega < -rs::Epsilon) angle = -angle;
	_motor[id].goal = _motor[id].theta + rs::D2R(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[0]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	RS_MUTEX_UNLOCK(&_goal_mutex);

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
	RS_MUTEX_LOCK(&_goal_mutex);

	// set new goal angles
	_motor[id].goal = rs::D2R(angle);

	// actively seeking an angle
	_motor[id].mode = SEEK;

	// enable motor
	RS_MUTEX_LOCK(&_theta_mutex);
	dJointEnable(_motor[id].id);
	dJointSetAMotorAngle(_motor[id].id, 0, _motor[id].theta);
	dBodyEnable(_body[0]);
	RS_MUTEX_UNLOCK(&_theta_mutex);

	// set success to false
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	_motor[id].success = false;
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// unlock goal
	RS_MUTEX_UNLOCK(&_goal_mutex);

	// success
	return 0;
}

int Robot::moveJointWait(int id) {
	// wait for motion to complete
	RS_MUTEX_LOCK(&_motor[id].success_mutex);
	while ( !_motor[id].success ) { RS_COND_WAIT(&_motor[id].success_cond, &_motor[id].success_mutex); }
	RS_MUTEX_UNLOCK(&_motor[id].success_mutex);

	// success
	return 0;
}

void Robot::pauseWait(void) {
	_sim->pauseWait();
}

#ifdef RS_RESEARCH
void Robot::setCPGGoal(double value) {
	_next_goal = value;
}
#endif

/**********************************************************
	protected functions
 **********************************************************/
int Robot::addToSim(dWorldID &world, dSpaceID &space, int id, Sim *sim) {
	this->setID(id);
	_world = world;
	_space = dHashSpaceCreate(space);
	_sim = sim;

	// success
	return 0;
}

int Robot::doze(double ms) {
#ifdef RS_WIN32
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

double Robot::getCenter(int i) {
	rs::Pos p2 = this->getRobotBodyPosition(0, this->getPosition(), this->getQuaternion());
	return p2[i];
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
	if ( fabs(R[8]-1) < rs::Epsilon ) {			// R_31 == 1; theta = rs::Pi/2
		angles[0] = atan2(-R[1], -R[2]);		// psi
		angles[1] = rs::Pi/2;					// theta
		angles[2] = 0;							// phi
	}
	else if ( fabs(R[8]+1) < rs::Epsilon ) {	// R_31 == -1; theta = -rs::Pi/2
		angles[0] = atan2(R[1], R[2]);			// psi
		angles[1] = -rs::Pi/2;					// theta
		angles[2] = 0;							// phi
	}
	else {
		angles[1] = asin(R[8]);
		angles[0] = atan2(R[9]/cos(angles[0]), R[10]/cos(angles[0]));
		angles[2] = atan2(R[4]/cos(angles[0]), R[0]/cos(angles[0]));
	}
	// convert to 0->2*PI
	if (angles[i] < 0) angles[i] = 2 * rs::Pi + angles[i];
	// return
	return angles[i];
}

double Robot::mod_angle(double past_ang, double cur_ang, double ang_rate) {
	double new_ang = 0;
	int stp = (int)( fabs(past_ang) / rs::Pi );
	double past_ang_mod = fabs(past_ang) - stp*rs::Pi;

	if ( (int)(ang_rate*1000) == 0 ) {
		new_ang = past_ang;
	}
	// positive angular velocity, positive angle
	else if ( ang_rate > 0 && past_ang >= 0 ) {
		// cross 180
		if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod + 2*rs::Pi); }
		// negative
		else if ( cur_ang < 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod + rs::Pi); }
		// cross 0
		else if ( cur_ang > 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod + rs::Pi); }
		// positive
		else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod); }
	}
	// positive angular velocity, negative angle
	else if ( ang_rate > 0 && past_ang < 0 ) {
		// cross 180
		if ( cur_ang < 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod + rs::Pi); }
		// negative
		else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod); }
		// cross 0
		else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod); }
		// positive
		else if ( cur_ang > 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod - rs::Pi); }
	}
	// negative angular velocity, positive angle
	else if ( ang_rate < 0 && past_ang >= 0 ) {
		// cross 180
		if ( cur_ang > 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod - rs::Pi); }
		// negative
		else if ( cur_ang < 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod + rs::Pi); }
		// cross 0
		else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod); }
		// positive
		else if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang - past_ang_mod); }
	}
	// negative angular velocity, negative angle
	else if ( ang_rate < 0 && past_ang < 0 ) {
		// cross 180
		if ( cur_ang > 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod - 2*rs::Pi); }
		// negative
		else if ( cur_ang < 0 && !(stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod); }
		// cross 0
		else if ( cur_ang < 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod - rs::Pi); }
		// positive
		else if ( cur_ang > 0 && (stp % 2) ) { new_ang = past_ang + (cur_ang + past_ang_mod - rs::Pi); }
	}

	return new_ang;
}

short Robot::noisy(float *a, short length, float sigma) {
	// initialize variables
	float *rand = new float[length];
	float sum = 0;

	if (length == 1)
		a[0] += this->normal(sigma);
	else {
		// compute magnitude of randomized vector
		for (int i = 0; i < length; i++) {
			rand[i] = this->normal(sigma);
			sum += (a[i] + rand[i]) * (a[i] + rand[i]);
		}
		float mag = sqrt(sum);

		// normalize vector
		for (short i = 0; i < length; i++) {
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
float Robot::normal(float sigma) {
	// compute pair of random uniform data
	float u1 = this->uniform();
	float u2 = this->uniform();

	// box-muller transform to gaussian
	return sigma*(sqrt(-2.0*log(u1))*cos(2*rs::Pi*u2));
}

float Robot::uniform(void) {
	int k = _seed/127773;
	_seed = 16807 * (_seed - k*127773) - k*2836;
	if (_seed < 0) _seed = _seed + 2147483647;
	return ((float)(_seed) * 4.656612875E-10);
}

