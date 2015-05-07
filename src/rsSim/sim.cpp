#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <rs/Macros>
#include <rsSim/Sim>

using namespace rsSim;

Sim::Sim(bool pause, bool rt) {
	// create ODE simulation space
	dInitODE2(0);										// initialized ode library
	_world = dWorldCreate();							// create world for simulation
	_space = dHashSpaceCreate(0);						// create space for robots
	_group = dJointGroupCreate(0);						// create group for joints
	dCreatePlane(_space, 0, 0, 1, 0);					// ground plane

	// simulation parameters
	dWorldSetAutoDisableFlag(_world, 1);				// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(_world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(_world, 0.01);	// linear velocity threshold
	dWorldSetAutoDisableSteps(_world, 4);				// number of steps below thresholds before stationary
	dWorldSetContactSurfaceLayer(_world, 0.01);			// depth each body can sink into another body before resting
	dWorldSetGravity(_world, 0, 0, -9.81);				// gravity

	// default collision parameters
	_friction[0] = 0.9;									// friction body/ground
	_friction[1] = 0.6;									// friction body/body
	_restitution[0] = 0.3;								// restitution body/ground
	_restitution[1] = 0.3;								// restitution body/body

	// simulation variables
	_clock = 0;											// start clock
	_collision = true;									// perform inter-robot collisions
	_pause = pause;										// start paused
	_rt = rt;											// real time
	_running = true;									// is simulation running
	_step = 0.004;										// initial time step
	_stop = 0;											// time at which to stop simulation

	// thread variables
	MUTEX_INIT(&_clock_mutex);
	MUTEX_INIT(&_pause_mutex);
	MUTEX_INIT(&_robot_mutex);
	MUTEX_INIT(&_running_mutex);
	MUTEX_INIT(&_step_mutex);
	COND_INIT(&_running_cond);
	THREAD_CREATE(&_simulation, (void* (*)(void *))&Sim::simulation_thread, this);
}

Sim::~Sim(void) {
	// remove simulation
	MUTEX_LOCK(&_running_mutex);
	_running = false;
	MUTEX_UNLOCK(&_running_mutex);
	THREAD_JOIN(_simulation);
	COND_DESTROY(&_running_cond);
	MUTEX_DESTROY(&_clock_mutex);
	MUTEX_DESTROY(&_pause_mutex);
	MUTEX_DESTROY(&_robot_mutex);
	MUTEX_DESTROY(&_running_mutex);
	MUTEX_DESTROY(&_step_mutex);

	// remove ode
	dJointGroupDestroy(_group);
	dSpaceDestroy(_space);
	dWorldDestroy(_world);
	dCloseODE();

	// remove robots
	for (unsigned int i = 0; i < _robot.size(); i++) {
		_robot.erase(_robot.begin() + i);
	}
}

/**********************************************************
	Public member functions
 **********************************************************/
int Sim::addRobot(rsSim::Robot *robot, int id, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, int ground) {
	// lock robot data
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(RobotNode());
	_robot.back().robot = robot;

	// give simulation data to robot
	robot->addToSim(_world, _space, id, this);

	// build
	robot->build(p, q, a, ground);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int Sim::addRobot(rsSim::ModularRobot *robot, int id, rsSim::Robot *base, const rs::Vec &a, int face1, int face2, int type, int side, int orientation, int ground) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(RobotNode());
	_robot.back().robot = robot;

	// give simulation data to robot
	robot->addToSim(_world, _space, id, this);

	// build
	rs::Quat Q = dynamic_cast<rsSim::ModularRobot*>(base)->getRobotBodyQuaternion(face1, base->getAngle(face1-1), base->getQuaternion());
	rs::Pos P = dynamic_cast<rsSim::ModularRobot*>(base)->getRobotFacePosition(face1, base->getPosition(), base->getQuaternion());
	int conn_orientation = dynamic_cast<rsSim::ModularRobot*>(base)->getConnectorOrientation(face1);
	P = dynamic_cast<rsSim::ModularRobot*>(base)->getConnFacePosition(type, side, conn_orientation, P, Q);
	Q = dynamic_cast<rsSim::ModularRobot*>(base)->getConnFaceQuaternion(type, side, conn_orientation, Q);
	robot->build(P, Q, a, dynamic_cast<rsSim::ModularRobot*>(base)->getConnectorBodyID(face1), face2, orientation, ground);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, double mass) {
	// create body
	dMass m;
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	if (mass == 0) {
		dBodyDisable(*body);
		mass = 1;
	}
	dBodySetPosition(*body, p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(*body, Q);

	// position geom
	dMassSetBoxTotal(&m, mass, l[0], l[1], l[2]);
	dGeomID geom = dCreateBox(_space, l[0], l[1], l[2]);
	dGeomSetBody(geom, *body);
	dGeomSetOffsetPosition(geom, -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(*body, &m);

	// return object
	return body;
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, double mass, int axis) {
	// create body
	dMass m;
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	if (mass == 0) {
		dBodyDisable(*body);
		mass = 1;
	}
	dBodySetPosition(*body, p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(*body, Q);

	// position geom
	dMassSetCylinderTotal(&m, mass, axis, l[0], l[1]);
	dGeomID geom = dCreateCylinder(_space, l[0], l[1]);
	dGeomSetBody(geom, *body);
	dGeomSetOffsetPosition(geom, -m.c[0], -m.c[1], -m.c[2]);
	dMatrix3 R;
	if (axis == 1) {		dRFromAxisAndAngle(R, 0, 1, 0, rs::PI/2); }
	else if (axis == 2) {	dRFromAxisAndAngle(R, 1, 0, 0, rs::PI/2); }
	else if (axis == 3) {	dRFromAxisAndAngle(R, 0, 0, 1, 0); }
	dGeomSetOffsetRotation(geom, R);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(*body, &m);

	// return object
	return body;
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Vec&l, double mass) {
	// create body
	dMass m;
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	if (mass == 0) {
		dBodyDisable(*body);
		mass = 1;
	}
	dBodySetPosition(*body, p[0], p[1], p[2]);

	// position geom
	dMassSetSphereTotal(&m, mass, l[0]);
	dGeomID geom = dCreateSphere(_space, l[0]);
	dGeomSetBody(geom, *body);
	dGeomSetOffsetPosition(geom, -m.c[0], -m.c[1], -m.c[2]);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(*body, &m);

	// return object
	return body;
}

int Sim::deleteRobot(int id) {
	// lock robot data to delete
	MUTEX_LOCK(&_robot_mutex);

	// find robot by id and remove
	for (unsigned int i = 0; i < _robot.size(); i++) {
		if (_robot[i].robot->getID() == id) {
			_robot.erase(_robot.begin() + i);
			break;
		}
	}

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return _robot.size();
}

void Sim::done(void) {
	COND_ACTION(&_running_cond, &_running_mutex, _running = false);
}

double Sim::getClock(void) {
	MUTEX_LOCK(&_clock_mutex);
	double clock = _clock;
	MUTEX_UNLOCK(&_clock_mutex);
	return clock;
}

int Sim::getCOR(double &robot, double &ground) {
	robot = _restitution[0];
	ground = _restitution[1];

	// success
	return 0;
}

void Sim::getCoM(double &x, double &y, double &z) {
	double com[3] = {0}, a, b, c;
	for (unsigned int i = 0; i < _robot.size(); i++) {
		_robot[i].robot->getCoM(a, b, c);
		com[0] += a;
		com[1] += b;
		com[2] += c;
	}
	x = com[0];
	y = com[1];
	z = com[2];
}

int Sim::getMu(double &robot, double &ground) {
	robot = _friction[0];
	ground = _friction[1];

	// success
	return 0;
}

bool Sim::getPause(void) {
	MUTEX_LOCK(&_pause_mutex);
	bool pause = _pause;
	MUTEX_UNLOCK(&_pause_mutex);
	return pause;
}

Robot* Sim::getRobot(int id) {
	for (unsigned int i = 0; i < _robot.size(); i++) {
		if (_robot[i].robot->getID() == id)
			return _robot[i].robot;
	}
	return NULL;
}

double Sim::getStep(void) {
	MUTEX_LOCK(&_step_mutex);
	double step = _step;
	MUTEX_UNLOCK(&_step_mutex);
	return step;
}

void Sim::mutexLock(int type) {
	MUTEX_LOCK(&_robot_mutex);
}

void Sim::mutexUnlock(int type) {
	MUTEX_UNLOCK(&_robot_mutex);
}

void Sim::run(int milliseconds, void (*output)(void), int interval) {
	// calculate sleep time
	if (!interval) { interval = milliseconds; }

	// start simulation
	this->start();

	if (_rt) {
		// sleep and run output
		for (int i = 0; i < milliseconds/interval; i++) {
			#ifdef _WIN32
				Sleep(interval);
			#else
				usleep(interval*1000);
			#endif
			if (output) output();
		}
		// end simulation
		this->stop();
	}
	else {
		// set stopping time
		_stop = milliseconds/1000.0;
		// wait for simulation loop to signal
		MUTEX_LOCK(&_running_mutex);
		while (_running) {
			COND_WAIT(&_running_cond, &_running_mutex);
		}
		MUTEX_UNLOCK(&_running_mutex);
	}
}

int Sim::setCollisions(int mode) {
	switch (mode) {
		case 0:
			_collision = false;
			break;
		case 1:
			_collision = true;
			break;
		case 2:
			_collision = _collision ? false : true;
			break;
	}

	// success
	return 0;
}

int Sim::setCOR(double robot, double ground) {
	_restitution[0] = robot;
	_restitution[1] = ground;

	// success
	return 0;
}

int Sim::setMu(double robot, double ground) {
	_friction[0] = robot;
	_friction[1] = ground;

	// success
	return 0;
}

int Sim::pause(int mode) {
	// lock pause
	MUTEX_LOCK(&_pause_mutex);

	// switch pause variable
	switch (mode) {
		case 0:
			_pause = false;
			break;
		case 1:
			_pause = true;
			break;
		case 2:
			_pause = _pause ? false : true;
			break;
	}

	// unlock pause
	MUTEX_UNLOCK(&_pause_mutex);

	// success
	return 0;
}

void Sim::start(void) {
	// unpause simulation
	MUTEX_LOCK(&_pause_mutex);
	_pause = false;
	MUTEX_UNLOCK(&_pause_mutex);
}

void Sim::stop(void) {
	// ask sim to stop running
	MUTEX_LOCK(&_running_mutex);
	_running = false;
	MUTEX_UNLOCK(&_running_mutex);

	// wait for last loop to finish
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);
}

/**********************************************************
	Private functions
 **********************************************************/
void* Sim::simulation_thread(void *arg) {
	// cast to type sim
	Sim *sim = (Sim *)arg;

	// initialize local variables
	int i, num = 20;
	unsigned int sum = 0, clock = 0, restart = 0;
	unsigned int *dt = new unsigned int[num]();
#ifdef _WIN32
	DWORD start_time = 0, start = 0, end = 0;
#else
	struct timespec s_time;
	unsigned int start_time = 0, start = 0, end = 0;
#endif

	MUTEX_LOCK(&(sim->_running_mutex));
	while (sim->_running) {
		MUTEX_UNLOCK(&(sim->_running_mutex));

		// lock pause variable
		MUTEX_LOCK(&(sim->_pause_mutex));

		if (sim->_rt) {
			// get starting times
#ifdef _WIN32
			start = GetTickCount();
#else
			clock_gettime(CLOCK_REALTIME, &s_time);
			start = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif
		}

		while (!(sim->_pause) && sim->_running) {
			// unlock pause variable
			MUTEX_UNLOCK(&(sim->_pause_mutex));

			if (sim->_rt) {
				// get start time of execution in milliseconds
#ifdef _WIN32
				start_time = GetTickCount();
#else
				clock_gettime(CLOCK_REALTIME, &s_time);
				start_time = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif
			}

			// perform pre-collision updates
			MUTEX_LOCK(&(sim->_robot_mutex));
			for (unsigned int j = 0; j < sim->_robot.size(); j++) {
				THREAD_CREATE(&(sim->_robot[j].thread), (void* (*)(void *))&rsSim::Robot::simPreCollisionThreadEntry, (void *)sim->_robot[j].robot);
			}
			for (unsigned int j = 0; j < sim->_robot.size(); j++) {
				THREAD_JOIN(sim->_robot[j].thread);
			}

			// perform ode update
			dSpaceCollide(sim->_space, sim, &sim->collision);
			dWorldStep(sim->_world, sim->_step);
			MUTEX_LOCK(&(sim->_clock_mutex));
			sim->_clock += sim->_step;
			MUTEX_UNLOCK(&(sim->_clock_mutex));
			dJointGroupEmpty(sim->_group);

			// perform post-collision updates
			for (unsigned int j = 0; j < sim->_robot.size(); j++) {
				THREAD_CREATE(&(sim->_robot[j].thread), (void* (*)(void *))&rsSim::Robot::simPostCollisionThreadEntry, (void *)sim->_robot[j].robot);
			}
			for (unsigned int j = 0; j < sim->_robot.size(); j++) {
				THREAD_JOIN(sim->_robot[j].thread);
			}
			MUTEX_UNLOCK(&(sim->_robot_mutex));

			// get ending time
			if (sim->_rt) {
#ifdef _WIN32
				end = GetTickCount();
#else
				clock_gettime(CLOCK_REALTIME, &s_time);
				end = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif

				// running mean of last four time steps
				if (!restart) {
					for (i = num-2; i >= 0; i--) { dt[i+1] = dt[i]; }
					dt[0] = end - start_time;
					for (i = 0; i < num; i++) { sum += dt[i]; }
					sum /= num;
				}
				// on restart, reset all time steps
				else {
					restart = 0;
					sum = dt[0];
					dt[0] = num;
					for (i = 1; i < num; i++) { dt[i] = 0; }
				}

				// lock step variable
				MUTEX_LOCK(&(sim->_step_mutex));

				// set next time step if calculations took longer than step
				if ( (end - start) > ((unsigned int)(sim->_clock*1000) - clock/1000) ) {
					sim->_step = ((end - start - ((unsigned int)(sim->_clock*1000) - clock/1000))/num + sum)/1000.0;
				}
				// sleep until clock time equals step time
				else {
					sim->_step = sum/1000.0;
#ifdef _WIN32
					Sleep((unsigned int)(sim->_clock*1000) - (end - start) - clock/1000);
#else
					usleep(sim->_clock*1000000 - ((end - start)*1000) - clock);
#endif
				}

				// make sure time step is large enough
				sim->_step = (sim->_step*1000 < 4) ? 0.004 : sim->_step;

				// unlock step variable
				MUTEX_UNLOCK(&(sim->_step_mutex));
			}

			// stop when clock has reached max time
			if (sim->_stop && sim->_clock >= sim->_stop) { sim->_running = 0; }

			// lock pause variable
			MUTEX_LOCK(&(sim->_pause_mutex));
		}
		// unlock pause variable
		MUTEX_UNLOCK(&(sim->_pause_mutex));

		if (sim->_rt) {
			// reset clock counters on pausing
			restart = 1;
			end = start;
			clock = (unsigned int)(sim->_clock*1000000);
		}

		// lock running mutex
		MUTEX_LOCK(&(sim->_running_mutex));
	}
	// unlock running variable
	MUTEX_UNLOCK(&(sim->_running_mutex));

	// signal waiting threads
	COND_SIGNAL(&(sim->_running_cond));

	// cleanup
	delete [] dt;

	// end
	return NULL;
}

void Sim::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	Sim *ptr = (Sim *)data;

	// get bodies of geoms
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);

	// if geom bodies are connected, do not intersect
	if ( b1 && b2 && dAreConnected(b1, b2) ) return;

	// do not collide spaces (robots) together
	if (!ptr->_collision && dGeomIsSpace(o1) && dGeomIsSpace(o2)) {
		return;
	}

	// special case for collision of spaces
	if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, ptr, &ptr->collision);
		if ( dGeomIsSpace(o1) )	dSpaceCollide((dSpaceID)o1, ptr, &ptr->collision);
		if ( dGeomIsSpace(o2) ) dSpaceCollide((dSpaceID)o2, ptr, &ptr->collision);
	}
	else {
		dContact contact[8] = {0};
		for ( int i = 0; i < dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact)); i++ ) {
			if ( dGeomGetSpace(o1) == ptr->_space || dGeomGetSpace(o2) == ptr->_space ) {
				contact[i].surface.mu = ptr->_friction[0];
				contact[i].surface.bounce = ptr->_restitution[0];
			}
			else {
				contact[i].surface.mu = ptr->_friction[1];
				contact[i].surface.bounce = ptr->_restitution[1];
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach(dJointCreateContact(ptr->_world, ptr->_group, contact + i), b1, b2);
		}
	}
}

