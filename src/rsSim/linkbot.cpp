#include <iostream>

#include <rsSim/Sim>
#include <rsSim/Linkbot>

using namespace rsSim;
using namespace rsLinkbot;

Linkbot::Linkbot(int form) : rsRobots::Robot(form) {
	// initialize parameters
	this->init_params();
}

Linkbot::~Linkbot(void) {
std::cerr << "rsSim/~Linkbot start" << std::endl;
	// delete mutexes
	for (int i = 0; i < _dof; i++) {
		MUTEX_DESTROY(&_motor[i].success_mutex);
		COND_DESTROY(&_motor[i].success_cond);
	}
std::cerr << "rsSim/~Linkbot end" << std::endl;
}

/**********************************************************
	inherited functions
 **********************************************************/
int Linkbot::addConnector(int type, int face, int orientation, double size, int side, int conn) {
	// get connector body position
	rs::Pos P1 = this->getRobotFacePosition(face, this->getPosition(), this->getQuaternion());
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, 0, this->getQuaternion());
	if (conn == -1) {
		P1 = this->getConnBodyPosition(type, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = this->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = this->getConnFaceQuaternion(type, side, orientation, Q1);
		P1 = this->getConnBodyPosition(type, P1, Q1);
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
		case FACEPLATE:
			this->build_faceplate(_conn.back());
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

void Linkbot::addForce(int body, double fx, double fy, double fz) {
	_f.push_back(std::make_tuple(body, fx, fy, fz));
	dBodyAddForce(_body[body], fx, fy, fz);
}

int Linkbot::build(const rs::Pos &p, const rs::Quat &q, const double *a, int ground) {
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

int Linkbot::build(const rs::Pos &p, const rs::Quat &q, const double *a, dBodyID base, int face, int orientation, int ground) {
	// new position & quaternion classes
	rs::Pos P(p);
	rs::Quat Q(q);

	// rotate for orientation
	Q.multiply(sin(0.5*1.570796*orientation), 0, 0, cos(0.5*1.570796*orientation));

	// get position of robot
	if (face == FACE1)
		P.add(Q.multiply(_body_width/2 + _face_depth, 0, 0));
	else if (face == FACE2)
		P.add(Q.multiply(_face_depth + _body_length, 0, 0));
	else if (face == FACE3)
		P.add(Q.multiply(_body_width/2 + _face_depth, 0, 0));

	// get quaternion of robot
	if (face == FACE1)
		Q.multiply(sin(DEG2RAD(0.5*a[JOINT1])), 0, 0, cos(DEG2RAD(0.5*a[JOINT1])));
	else if (face == FACE2) {
		Q.multiply(0, 0, sin(-0.785398), cos(-0.785398));
		Q.multiply(sin(DEG2RAD(0.5*a[JOINT2])), 0, 0, cos(DEG2RAD(0.5*a[JOINT2])));
	}
	else if (face == FACE3) {
		Q.multiply(0, 0, sin(1.570796), cos(1.570796));
		Q.multiply(sin(DEG2RAD(-0.5*a[JOINT3])), 0, 0, cos(DEG2RAD(-0.5*a[JOINT3])));
	}

    // build new module
	this->buildIndividual(P, Q, a);

    // add fixed joint to attach two modules
	this->fix_body_to_connector(base, face);

	// fix to ground
	if (ground != -1) this->fixBodyToGround(_body[ground]);

	// success
	return 0;
}

int Linkbot::buildIndividual(const rs::Pos &p, const rs::Quat &q, const double *a) {
	// init body parts
	for (int i = 0; i < NUM_PARTS; i++) {
		_body.push_back(dBodyCreate(_world));
	}

	// convert input angles to radians
	for (int i = 0; i < _dof; i++) {
		_motor[i].goal = _motor[i].theta = DEG2RAD(a[i]);
	}
	_motor[_disabled].goal = _motor[_disabled].theta = 0;

	// build robot bodies
	this->build_body(p, q);
	this->build_face(FACE1, this->getRobotBodyPosition(FACE1, p, q), this->getRobotBodyQuaternion(FACE1, 0, q));
	this->build_face(FACE2, this->getRobotBodyPosition(FACE2, p, q), this->getRobotBodyQuaternion(FACE2, 0, q));
	this->build_face(FACE3, this->getRobotBodyPosition(FACE3, p, q), this->getRobotBodyQuaternion(FACE3, 0, q));

	// joint variable
	dJointID joint[_dof];
	rs::Pos o;

	// joint for body to face 1
	joint[JOINT1] = dJointCreateHinge(_world, 0);
	dJointAttach(joint[JOINT1], _body[BODY], _body[FACE1]);
	o = q.multiply(-_body_width/2, 0, 0);
	dJointSetHingeAnchor(joint[JOINT1], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
	o = q.multiply(1, 0, 0);
	dJointSetHingeAxis(joint[JOINT1], o[0], o[1], o[2]);
	dBodySetFiniteRotationAxis(_body[FACE1], o[0], o[1], o[2]);

	// joint for body to face 2
	if (_disabled == JOINT2) {
		joint[JOINT2] = dJointCreateFixed(_world, 0);
		dJointAttach(joint[JOINT2], _body[BODY], _body[FACE2]);
		dJointSetFixed(joint[JOINT2]);
	}
	else {
		joint[JOINT2] = dJointCreateHinge(_world, 0);
		dJointAttach(joint[JOINT2], _body[BODY], _body[FACE2]);
		o = q.multiply(0, -_body_width/2, 0);
		dJointSetHingeAnchor(joint[JOINT2], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(0, 1, 0);
		dJointSetHingeAxis(joint[JOINT2], o[0], o[1], o[2]);
		dBodySetFiniteRotationAxis(_body[FACE2], o[0], o[1], o[2]);
	}

	// joint for body to face 3
	if (_disabled == JOINT3) {
		joint[JOINT3] = dJointCreateFixed(_world, 0);
		dJointAttach(joint[JOINT3], _body[BODY], _body[FACE3]);
		dJointSetFixed(joint[JOINT3]);
	}
	else {
		joint[JOINT3] = dJointCreateHinge(_world, 0);
		dJointAttach(joint[JOINT3], _body[BODY], _body[FACE3]);
		o = q.multiply(-_body_width/2, 0, 0);
		dJointSetHingeAnchor(joint[JOINT3], o[0] + p[0], o[1] + p[1], o[2] + p[2]);
		o = q.multiply(-1, 0, 0);
		dJointSetHingeAxis(joint[JOINT3], o[0], o[1], o[2]);
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
		_motor[i].joint = joint[i];
		dJointAttach(_motor[i].id, _body[BODY], _body[FACE1 + i]);
		dJointSetAMotorMode(_motor[i].id, dAMotorUser);
		dJointSetAMotorNumAxes(_motor[i].id, 1);
		dJointSetAMotorAngle(_motor[i].id, 0, 0);
		dJointSetAMotorParam(_motor[i].id, dParamFMax, _motor[i].tau_max);
		dJointSetAMotorParam(_motor[i].id, dParamFudgeFactor, 0.3);
		dJointDisable(_motor[i].id);
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

double Linkbot::getAngle(int id) {
	if (id == _disabled)
		_motor[id].theta = 0;
	else
		_motor[id].theta = mod_angle(_motor[id].theta, dJointGetHingeAngle(_motor[id].joint), dJointGetHingeAngleRate(_motor[id].joint)) - _motor[id].offset;

    return _motor[id].theta;
}

void Linkbot::getCoM(double &x, double &y, double &z) {
	dMass m[NUM_PARTS];
	double total = 0;
	x = 0, y = 0, z = 0;
	for (int i = 0; i < NUM_PARTS; i++) {
		dBodyGetMass(_body[i], &m[i]);
	}
	for (int i = 0; i < NUM_PARTS; i++) {
		const double *p = dBodyGetPosition(_body[i]);
		x += m[i].mass*p[0];
		y += m[i].mass*p[1];
		z += m[i].mass*p[2];
		total += m[i].mass;
	}
	x /= total;
	y /= total;
	z /= total;
}

void Linkbot::init_params(void) {
	_dof = NUM_JOINTS;

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

void Linkbot::simPreCollisionThread(void) {
	// lock angle and goal
	MUTEX_LOCK(&_goal_mutex);
	MUTEX_LOCK(&_theta_mutex);

	// apply forces
	for (int i = 0; i < _f.size(); i++) {
		dBodyAddForce(_body[std::get<0>(_f[i])], std::get<1>(_f[i]), std::get<2>(_f[i]), std::get<3>(_f[i]));
	}

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
			if (_conn[k].face == i+1)
				dBodySetFiniteRotationAxis(_conn[k].body, axis[0], axis[1], axis[2]);
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
	dMassTranslate(&m, 8*_conn_depth, 0, -_body_height/2);

	// build geometries
	dGeomID geom[4];

	// set geometry 0 - box
	geom[0] = dCreateBox(_space, _conn_depth, 1.5*_face_radius, _body_height);
	dGeomSetBody(geom[0], conn.body);

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
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.0368 - m.c[0], -m.c[1], -_body_height/2 - 0.005 - m.c[2]);
	}
	// custom drawn one for mathematics
	else {
		// set geometry 1 - horizontal support
		geom[1] = dCreateBox(_space, 0.02, 0.022, 0.0032);
		dGeomSetBody(geom[1], conn.body);
		dGeomSetOffsetPosition(geom[1], _conn_depth/2 + 0.01 - m.c[0], -m.c[1], -_body_height/2 + 0.0016 - m.c[2]);

		// set geometry 2 - ball support
		geom[2] = dCreateCylinder(_space, 0.011, _radius -_face_radius - 0.006 + 0.0032);
		dGeomSetBody(geom[2], conn.body);
		dGeomSetOffsetPosition(geom[2], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 - (_radius -_face_radius - 0.006)/2 + 0.0016 - m.c[2]);

		// set geometry 3 - sphere
		geom[3] = dCreateSphere(_space, 0.006);
		dGeomSetBody(geom[3], conn.body);
		dGeomSetOffsetPosition(geom[3], _conn_depth/2 + 0.02 - m.c[0], -m.c[1], -_body_height/2 + _face_radius - _radius + 0.006 - m.c[2]);
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

