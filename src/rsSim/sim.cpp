#include <rsSim/sim.hpp>

using namespace rsSim;

Sim::Sim(int pause) {
	init_ode();
	init_sim(pause);
}

Sim::~Sim(void) {
std::cerr << "deleting Sim" << std::endl;
	// remove simulation
	MUTEX_LOCK(&_running_mutex);
	_running = 0;
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
	for (int i = 0; i < _robot.size(); i++) {
		delete _robot[i];
	}
}

/**********************************************************
	Initialization functions
 **********************************************************/
int Sim::init_ode(void) {
	// create ODE simulation space
	dInitODE2(0);										// initialized ode library
	_world = dWorldCreate();							// create world for simulation
	_space = dHashSpaceCreate(0);						// create space for robots
	_group = dJointGroupCreate(0);						// create group for joints
	dGeomID geom = dCreatePlane(_space, 0, 0, 1, 0);	// ground plane

	// simulation parameters
	dWorldSetAutoDisableFlag(_world, 1);				// auto-disable bodies that are not moving
	dWorldSetAutoDisableAngularThreshold(_world, 0.01);	// threshold velocity for defining movement
	dWorldSetAutoDisableLinearThreshold(_world, 0.01);	// linear velocity threshold
	dWorldSetAutoDisableSteps(_world, 4);				// number of steps below thresholds before stationary
	dWorldSetContactSurfaceLayer(_world, 0.01);			// depth each body can sink into another body before resting
	dWorldSetGravity(_world, 0, 0, -9.81);				// gravity

	// success
	return 0;
}

int Sim::init_sim(int pause) {
	// default collision parameters
	_mu[0] = 0.9;	_mu[1] = 0.6;
	_cor[0] = 0.3;	_cor[1] = 0.3;

	// thread variables
	MUTEX_INIT(&_clock_mutex);
	MUTEX_INIT(&_pause_mutex);
	MUTEX_INIT(&_robot_mutex);
	MUTEX_INIT(&_running_mutex);
	MUTEX_INIT(&_step_mutex);
	COND_INIT(&_running_cond);
	THREAD_CREATE(&_simulation, (void* (*)(void *))&Sim::simulation_thread, this);

	// variables to keep track of progress of simulation
	_running = 1;			// is simulation running
	_pause = 1;				// start paused
    _step = 0.004;			// initial time step
	_clock = 0;				// start clock
	_collision = true;		// perform inter-robot collisions

	// success
	return 0;
}

/**********************************************************
	Public member functions
 **********************************************************/
/*int Sim::addRobot(Robot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(new Robots());
	_robot.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// find next robot in xml list
	int i = 0;
	for (i = 0; i < _xmlbot.size(); i++) {
		if (_xmlbot[i]->type == form && !_xmlbot[i]->connected)
			break;
	}

	// no robot found
	if (i == _xmlbot.size() || _xmlbot[i]->type != form) {
		switch (form) {
			case NXT:
				fprintf(stderr, "Error: Could not find NXT in Sim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, _xmlbot[i]->id, _robot.size()-1, this);

	// check if robot is colliding with others
	for (int j = 0; j < _robot.size() - 1; j++) {
		if ( (fabs(_robot[j]->robot->getCenter(0) - _xmlbot[i]->x) < 0.1) && (fabs(_robot[j]->robot->getCenter(1) - _xmlbot[i]->y) < 0.1) ) {
			fprintf(stderr, "Warning: Robot %d and Robot %d are possibly colliding.\n", _robot[j]->robot->getID() + 1, _xmlbot[i]->id + 1);
			fprintf(stderr, "         Please check Sim GUI for x and y positions that may be too close.\n");
		}
	}

	// build robot
	robot->build(_xmlbot[i]);

	// robot now connected
	_xmlbot[i]->connected = 1;

#ifdef ENABLE_GRAPHICS
	//if (_lib) _robot.back()->node = _graphics->drawRobot(robot, _xmlbot[i], form, _xmlbot[i]->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}*/

/*int Sim::addRobot(ModularRobot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(new Robots());
	_robot.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// find next robot in xml list
	int i;
	for (i = 0; i < _xmlbot.size(); i++) {
		if (_xmlbot[i]->type == form && !_xmlbot[i]->connected)
			break;
	}

	// no robot found
	if (i == _xmlbot.size() || _xmlbot[i]->type != form) {
		switch (form) {
			case MOBOT:
				fprintf(stderr, "Error: Could not find Mobot in Sim GUI.\n");
				break;
			case LINKBOTI:
				fprintf(stderr, "Error: Could not find LinkbotI in Sim GUI.\n");
				break;
			case LINKBOTL:
				fprintf(stderr, "Error: Could not find LinkbotL in Sim GUI.\n");
				break;
			case LINKBOTT:
				fprintf(stderr, "Error: Could not find LinkbotT in Sim GUI.\n");
				break;
			case CUBUS:
				fprintf(stderr, "Error: Could not find CUBUS in Sim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, _xmlbot[i]->id, _robot.size()-1, this);

	// find if robot is connected to another one
	int j;
	for (j = 0; j < _xmlbot[i]->conn.size(); j++) {
		if (_xmlbot[i]->conn[j]->robot != _xmlbot[i]->id)
			break;
	}

	// if robot is connected to another one
	if (j != _xmlbot[i]->conn.size()) {
		for (int k = 0; k < _robot.size(); k++) {
			if (_robot[k]->robot->getID() == _xmlbot[i]->conn[j]->robot) {
				ModularRobot *r = dynamic_cast<ModularRobot *>(_robot[k]->robot);
				dBodyID body = r->getConnectorBodyID(_xmlbot[i]->conn[j]->face1);
				dMatrix3 R;
				double m[3] = {0};
				r->getFaceParams(_xmlbot[i]->conn[j]->face1, R, m);
				robot->build(_xmlbot[i], R, m, body, _xmlbot[i]->conn[j]);
				robot->addNeighbor(r, _xmlbot[i]->conn[j]->face2-1, _xmlbot[i]->conn[j]->face1-1);
				r->addNeighbor(robot, _xmlbot[i]->conn[j]->face1-1, _xmlbot[i]->conn[j]->face2-1);
				break;
			}
		}
	}
	else {
		// check if robot is colliding with others
		for (int k = 0; k < _robot.size() - 1; k++) {
			if ( (fabs(_robot[k]->robot->getCenter(0) - _xmlbot[i]->x) < 0.1) && (fabs(_robot[k]->robot->getCenter(1) - _xmlbot[i]->y) < 0.1) ) {
				std::cerr << "WARNING: Robot " << _robot[k]->robot->getID() + 1 << "and Robot " << _xmlbot[i]->id + 1 << " are possibly colliding." << std::endl;
				std::cerr << "         Please check Sim GUI for x and y positions that may be too close." << std::endl;
			}
		}
		// build
		robot->build(_xmlbot[i]);
	}

	// robot now connected
	_xmlbot[i]->connected = 1;

#ifdef ENABLE_GRAPHICS
	//if (_lib) _robot.back()->node = _graphics->drawRobot(robot, _xmlbot[i], form, _xmlbot[i]->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}*/

/*int Sim::addRobot2(ModularRobot *robot, rsXML::Robot *bot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(new Robots());
	_robot.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// give simulation data to robot
	robot->addToSim(_world, _space, bot->id, _robot.size()-1, this);

	// build
	robot->build(bot);

#ifdef ENABLE_GRAPHICS
	//_robot.back()->node = _graphics->drawRobot(robot, bot, form, bot->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}*/

/*int Sim::addRobot2(ModularRobot *robot, rsXML::Robot *bot, ModularRobot *base) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(new Robots());
	_robot.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// give simulation data to robot
	robot->addToSim(_world, _space, bot->id, _robot.size()-1, this);

	// build
	dBodyID body = base->getConnectorBodyID(bot->conn[0]->face1);
	dMatrix3 R;
	double m[3] = {0};
	base->getFaceParams(bot->conn[0]->face1, R, m);
	robot->build(bot, R, m, body, bot->conn[0]);

#ifdef ENABLE_GRAPHICS
	//_robot.back()->node = _graphics->drawRobot(robot, bot, form, bot->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}*/

int Sim::addRobot3(rsSim::ModularRobot *robot, int id, const double *p, const double *r, const double *a, int ground, int trace) {
	// lock robot data
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robot.push_back(new Robots());
	_robot.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// give simulation data to robot
	robot->addToSim(_world, _space, id, _robot.size()-1, this);

	// build
	robot->build(id, p, r, a, ground);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

Ground2* Sim::addGround(const double *p, const double *q, const double *l, double mass) {
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

Ground2* Sim::addGround(const double *p, const double *q, const double *l, double mass, int axis) {
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
	if (axis == 1) {		dRFromAxisAndAngle(R, 0, 1, 0, M_PI/2); }
	else if (axis == 2) {	dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2); }
	else if (axis == 3) {	dRFromAxisAndAngle(R, 0, 0, 1, 0); }
	dGeomSetOffsetRotation(geom, R);

	// set mass center to (0,0,0) of _bodyID
	dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
	dBodySetMass(*body, &m);

	// return object
	return body;
}

Ground2* Sim::addGround(const double *p, const double *l, double mass) {
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

int Sim::deleteRobot(int loc) {
	// lock robot data to delete
	MUTEX_LOCK(&_robot_mutex);

	// remove robot
	_robot.erase(_robot.begin()+loc);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	if (!_robot.size())
		return 0;
	else
		return 1;
}

void Sim::done(void) {
	SIGNAL(&_running_cond, &_running_mutex, _running = 0);
}

double Sim::getClock(void) {
	MUTEX_LOCK(&_clock_mutex);
	double clock = _clock;
	MUTEX_UNLOCK(&_clock_mutex);
	return clock;
}

int Sim::getCOR(double &robot, double &ground) {
	robot = _cor[0];
	ground = _cor[1];

	// success
	return 0;
}

int Sim::getMu(double &robot, double &ground) {
	robot = _mu[0];
	ground = _mu[1];

	// success
	return 0;
}

int Sim::getPause(void) {
	MUTEX_LOCK(&_pause_mutex);
	int pause = _pause;
	MUTEX_UNLOCK(&_pause_mutex);
	return pause;
}

double Sim::getStep(void) {
	MUTEX_LOCK(&_step_mutex);
	double step = _step;
	MUTEX_UNLOCK(&_step_mutex);
	return step;
}

// TODO: implement function
int Sim::getUnits(void) {
	return 0;
}

int Sim::runSimulation(void) {
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);

	// success
	return 0;
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
	_cor[0] = robot;
	_cor[1] = ground;

	// success
	return 0;
}

int Sim::setMu(double robot, double ground) {
	_mu[0] = robot;
	_mu[1] = ground;

	// success
	return 0;
}

int Sim::setPause(int mode) {
	// lock pause
	MUTEX_LOCK(&_pause_mutex);

	// switch pause variable
	switch (mode) {
		case 0:
			_pause = 0;
			break;
		case 1:
			_pause = 1;
			break;
		case 2:
			_pause = _pause ? 0: 1;
			break;
	}

	// unlock pause
	MUTEX_UNLOCK(&_pause_mutex);

	// success
	return 0;
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
			for (int j = 0; j < sim->_robot.size(); j++) {
				THREAD_CREATE(&(sim->_robot[j]->thread), (void* (*)(void *))&Robot::simPreCollisionThreadEntry, (void *)sim->_robot[j]->robot);
			}
			for (int j = 0; j < sim->_robot.size(); j++) {
				THREAD_JOIN(sim->_robot[j]->thread);
			}

			// perform ode update
			dSpaceCollide(sim->_space, sim, &sim->collision);
			dWorldStep(sim->_world, sim->_step);
			MUTEX_LOCK(&(sim->_clock_mutex));
			sim->_clock += sim->_step;
			MUTEX_UNLOCK(&(sim->_clock_mutex));
			dJointGroupEmpty(sim->_group);

			// perform post-collision updates
			for (int j = 0; j < sim->_robot.size(); j++) {
				THREAD_CREATE(&(sim->_robot[j]->thread), (void* (*)(void *))&Robot::simPostCollisionThreadEntry, (void *)sim->_robot[j]->robot);
			}
			for (int j = 0; j < sim->_robot.size(); j++) {
				THREAD_JOIN(sim->_robot[j]->thread);
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

std::cerr << "end sim thread" << std::endl;
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
				contact[i].surface.mu = ptr->_mu[0];
				contact[i].surface.bounce = ptr->_cor[0];
			}
			else {
				contact[i].surface.mu = ptr->_mu[1];
				contact[i].surface.bounce = ptr->_cor[1];
			}
			contact[i].surface.mode = dContactBounce | dContactApprox1;
			dJointAttach( dJointCreateContact(ptr->_world, ptr->_group, contact + i), b1, b2);
		}
	}
}

