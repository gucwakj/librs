#include <config.h>
#include <rs/Macros>
#include <rs/Timer>
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
	public functions
 **********************************************************/
void Sim::addRobot(rsSim::Robot *robot, short id, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &w, short ground) {
	// lock robot data
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(RobotNode());
	_robot.back().robot = robot;

	// give simulation data to robot
	robot->addToSim(_world, _space, id, this);

	// build
	robot->build(p, q, a, w, ground);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);
}

void Sim::addRobot(rsSim::ModularRobot *robot, short id, rsSim::Robot *base, const rs::Vec &a, short face1, short face2, short type, short side, short orientation, short ground) {
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
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, double mass) {
	// body
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	dBodySetPosition(*body, p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(*body, Q);

	// mass
	dMass m;
	dMassSetBoxTotal(&m, mass, l[0], l[1], l[2]);
	dBodySetMass(*body, &m);

	// geom
	dGeomID geom = dCreateBox(_space, l[0], l[1], l[2]);
	dGeomSetBody(geom, *body);

	// return object
	return body;
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Quat &q, const rs::Vec &l, double mass, int axis) {
	// create body
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	dBodySetPosition(*body, p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(*body, Q);

	// mass
	dMass m;
	dMassSetCylinderTotal(&m, mass, axis, l[0], l[1]);
	dBodySetMass(*body, &m);

	// geom
	dGeomID geom = dCreateCylinder(_space, l[0], l[1]);
	dGeomSetBody(geom, *body);
	dMatrix3 R;
	if (axis == 1) {		dRFromAxisAndAngle(R, 0, 1, 0, rs::Pi/2); }
	else if (axis == 2) {	dRFromAxisAndAngle(R, 1, 0, 0, rs::Pi/2); }
	else if (axis == 3) {	dRFromAxisAndAngle(R, 0, 0, 1, 0); }
	dGeomSetOffsetRotation(geom, R);

	// return object
	return body;
}

Obstacle* Sim::addObstacle(const rs::Pos &p, const rs::Vec &l, double mass) {
	// create body
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	dBodySetPosition(*body, p[0], p[1], p[2]);

	// mass
	dMass m;
	dMassSetSphereTotal(&m, mass, l[0]);
	dBodySetMass(*body, &m);

	// position geom
	dGeomID geom = dCreateSphere(_space, l[0]);
	dGeomSetBody(geom, *body);

	// return object
	return body;
}

Obstacle* Sim::addPullupBar(const rs::Pos &p, const rs::Quat &q, const rs::Vec &l) {
	// body
	dBodyID *body = new dBodyID();
	*body = dBodyCreate(_world);
	dBodySetPosition(*body, p[0], p[1], p[2]);
	dQuaternion Q = {q[3], q[0], q[1], q[2]};
	dBodySetQuaternion(*body, Q);

	// mass
	dMass m;
	dMassSetBoxTotal(&m, 1000, l[0], l[1], l[2]);
	dBodySetMass(*body, &m);

	// geom
	dGeomID geom[5];
	dMatrix3 R;

	// front left
	geom[0] = dCreateCylinder(_space, 0.0125, 0.16);
	dGeomSetBody(geom[0], *body);
	dGeomSetOffsetPosition(geom[0], -0.056569, -0.08, 0.056569);
	dRFromAxisAndAngle(R, 0, 1, 0, -rs::Pi/4);
	dGeomSetOffsetRotation(geom[0], R);

	// front right
	geom[1] = dCreateCylinder(_space, 0.0125, 0.16);
	dGeomSetBody(geom[1], *body);
	dGeomSetOffsetPosition(geom[1], 0.056569, -0.08, 0.056569);
	dRFromAxisAndAngle(R, 0, 1, 0, rs::Pi/4);
	dGeomSetOffsetRotation(geom[1], R);

	// back left
	geom[2] = dCreateCylinder(_space, 0.0125, 0.16);
	dGeomSetBody(geom[2], *body);
	dGeomSetOffsetPosition(geom[2], -0.056569, 0.08, 0.056569);
	dRFromAxisAndAngle(R, 0, 1, 0, -rs::Pi/4);
	dGeomSetOffsetRotation(geom[2], R);

	// back right
	geom[3] = dCreateCylinder(_space, 0.0125, 0.16);
	dGeomSetBody(geom[3], *body);
	dGeomSetOffsetPosition(geom[3], 0.056569, 0.08, 0.056569);
	dRFromAxisAndAngle(R, 0, 1, 0, rs::Pi/4);
	dGeomSetOffsetRotation(geom[3], R);

	// top
	geom[4] = dCreateCylinder(_space, 0.0125, 0.16);
	dGeomSetBody(geom[4], *body);
	dGeomSetOffsetPosition(geom[4], 0.056569, 0.08, 0.056569);
	dRFromAxisAndAngle(R, 1, 0, 0, rs::Pi/2);
	dGeomSetOffsetRotation(geom[4], R);

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

float Sim::getClock(void) {
	MUTEX_LOCK(&_clock_mutex);
	double clock = _clock;
	MUTEX_UNLOCK(&_clock_mutex);
	return clock;
}

void Sim::getCoM(double &x, double &y, double &z) {
	double total = 0;
	x = 0; y = 0; z = 0;
	for (unsigned int i = 0; i < _robot.size(); i++) {
		double m;
		rs::Pos p = _robot[i].robot->getCoM(m);
		x += m*p[0];
		y += m*p[1];
		z += m*p[2];
		total += m;
	}
	x /= total;
	y /= total;
	z /= total;
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

bool Sim::getRunning(void) {
	MUTEX_LOCK(&_running_mutex);
	bool running = _running;
	MUTEX_UNLOCK(&_running_mutex);
	return running;
}

float Sim::getStep(void) {
	MUTEX_LOCK(&_step_mutex);
	float step = _step;
	MUTEX_UNLOCK(&_step_mutex);
	return step;
}

void Sim::mutexLock(int type) {
	switch (type) {
		case Sim::AddRobot:
			MUTEX_LOCK(&_robot_mutex);
			break;
		case Sim::PauseSimulation:
			MUTEX_LOCK(&_pause_mutex);
			break;
		case Sim::RunningSimulation:
			MUTEX_LOCK(&_running_mutex);
			break;
	}
}

void Sim::mutexUnlock(int type) {
	switch (type) {
		case Sim::AddRobot:
			MUTEX_UNLOCK(&_robot_mutex);
			break;
		case Sim::PauseSimulation:
			MUTEX_UNLOCK(&_pause_mutex);
			break;
		case Sim::RunningSimulation:
			MUTEX_UNLOCK(&_running_mutex);
			break;
	}
}

void Sim::setPause(int mode) {
	// lock pause
	MUTEX_LOCK(&_pause_mutex);

	if (mode == 0)
		_pause = false;
	else if (mode == 1)
		_pause = true;
	else if (mode == 2)
		_pause = _pause ? false : true;

	// unlock pause
	MUTEX_UNLOCK(&_pause_mutex);
}

void Sim::run(unsigned int time) {
	// start simulation
	MUTEX_LOCK(&_pause_mutex);
	_pause = false;
	MUTEX_UNLOCK(&_pause_mutex);

	if (_rt) {
		// sleep
		rs::Timer timer(rs::Timer::MilliSeconds);
		timer.sleep(time);

		// ask sim to stop running
		MUTEX_LOCK(&_running_mutex);
		_running = false;
		MUTEX_UNLOCK(&_running_mutex);
	}
	else {
		// set stopping time
		_stop = time/1000.0;
	}

	// wait for simulation loop to signal
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);
}

#ifdef RS_RESEARCH
void Sim::setCPG(int (*function)(double, const double[], double[], void*), struct rsResearch::Params *params) {
	_integ.setup(function, params, _step);
}
#endif

void Sim::setCollisions(int mode) {
	if (mode == 0)
		_collision = false;
	else if (mode == 1)
		_collision = true;
	else if (mode == 2)
		_collision = _collision ? false : true;
}

/**********************************************************
	private functions
 **********************************************************/
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

void* Sim::simulation_thread(void *arg) {
	// cast to type sim
	Sim *sim = (Sim *)arg;

	// initialize local variables
	int i, num = 20;
	unsigned int sum = 0, clock = 0, restart = 0;
	unsigned int *dt = new unsigned int[num]();
#ifdef RS_WIN32
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
#ifdef RS_WIN32
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
#ifdef RS_WIN32
				start_time = GetTickCount();
#else
				clock_gettime(CLOCK_REALTIME, &s_time);
				start_time = s_time.tv_sec*1000 + s_time.tv_nsec/1000000;
#endif
			}

#ifdef RS_RESEARCH
			// research: cpg calculation
			rs::Vec v = sim->_integ.runStep(sim->_clock + sim->_step);
			for (unsigned int j = 0; j < sim->_robot.size(); j++) {
				sim->_robot[j].robot->setCPGGoal(v[j]);
			}
#endif

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
#ifdef RS_WIN32
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
#ifdef RS_WIN32
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

