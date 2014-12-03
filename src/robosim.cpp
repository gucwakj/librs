#include "robosim.hpp"

// global robot simulation object
//RoboSim *g_sim = NULL;

RoboSim::RoboSim(char *name, int pause, int lib) {
	_lib = lib;
#ifdef ENABLE_GRAPHICS
	//if (_lib) _graphics = new Graphics(this);
#endif // ENABLE_GRAPHICS
	init_ode();
	//init_xml(name);
	init_sim(pause);
#ifdef ENABLE_GRAPHICS
	if (_lib) {
		//_graphics->start(_pause);
		for (int i = 1; i < _ground.size(); i++) {
			//_graphics->drawGround(_ground[i]);
			i = i;
		}
	}
#endif // ENABLE_GRAPHICS
}

RoboSim::~RoboSim(void) {
#ifdef ENABLE_GRAPHICS
	//if (_lib) delete _graphics;
#endif // ENABLE_GRAPHICS

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

	// remove ground
	for (int i = 0; i < _ground.size(); i++) {
		delete _ground[i];
	}

	// remove robots
	for (int i = 0; i < _robots.size(); i++) {
		delete _robots[i];
	}

	// remove bot + connector list
	for (int i = 0; i < _xmlbot.size(); i++) {
		for (int j = 0; j < _xmlbot[i]->conn.size(); j++) {
			delete _xmlbot[i]->conn[j];
		}
		delete _xmlbot[i];
	}
}

/**********************************************************
	Initialization functions
 **********************************************************/
int RoboSim::init_ode(void) {
	// create ODE simulation space
	dInitODE2(0);										// initialized ode library
	_world = dWorldCreate();							// create world for simulation
	_space = dHashSpaceCreate(0);						// create space for robots
	_group = dJointGroupCreate(0);						// create group for joints
	_ground.push_back(new Ground());
	_ground[0]->body = NULL;							// immovable
	_ground[0]->geom = dCreatePlane(_space, 0, 0, 1, 0);// ground plane

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

int RoboSim::init_sim(int pause) {
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
	THREAD_CREATE(&_simulation, (void* (*)(void *))&RoboSim::simulation_thread, this);

	// variables to keep track of progress of simulation
	_running = 1;			// is simulation running
#ifdef ENABLE_GRAPHICS
	if (pause == 0) { _pause = 0; }
	else if (pause == 1) { _pause = 1; }
	else if (_pause == 3 && pause == 2) { _pause = 0; }
#else
	_pause = 0;				// do not start paused w/o graphics
#endif // ENABLE_GRAPHICS
    _step = 0.004;			// initial time step
	_clock = 0;				// start clock
	_collision = true;		// perform inter-robot collisions

	// success
	return 0;
}

/*int RoboSim::init_xml(char *name) {
	// initialize variables
	int *rtmp, *ftmp, *ntmp, *atmp, ctype = 0, cnum = 0;
	int tracking = 0;
	int custom = 0;
	double size = 0;
	_preconfig = 0;
	_pause = 3;
	_rt = 1;
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;

	// load xml config file
	tinyxml2::XMLDocument doc;
	char path[512];
#ifdef _WIN32
	if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path))) {
		strcat(path, "\\robosimrc");
	}
#else
	strcpy(path, getenv("HOME"));
	if (name) {
		FILE *fp = fopen(name, "r");
		if (fp) {
			strcpy(path, name);
			fclose(fp);
		}
		else {
			strcat(path, "/.robosimrc");
		}
	}
	else {
		strcat(path, "/.robosimrc");
	}
#endif
	int output = doc.LoadFile(path);
	if (output) {
		fprintf(stderr, "Error: Could not find RoboSim config file.\nPlease run RoboSim GUI.\n");
		exit(-1);
	}

	// check for custom mu params
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("mu")) ) {
		node->QueryDoubleAttribute("ground", &(_mu[0]));
		node->QueryDoubleAttribute("body", &(_mu[1]));
	}

	// check for custom cor params
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("cor")) ) {
		node->QueryDoubleAttribute("ground", &(_cor[0]));
		node->QueryDoubleAttribute("body", &(_cor[1]));
	}

	// check if should start paused
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("pause")) ) {
		node->QueryIntAttribute("val", &_pause);
	}

	// check if should run in real time
	if ( (node = doc.FirstChildElement("config")->FirstChildElement("realtime")) ) {
		node->QueryIntAttribute("val", &_rt);
	}

	// check for existence of ground node
	if ( (node = doc.FirstChildElement("ground")) ) {
		node = node->FirstChildElement();
	}

	// loop over all ground nodes
	while (node) {
		if ( !strcmp(node->Value(), "box") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->type = BOX;
			_ground.back()->r = 0;
			_ground.back()->g = 0;
			_ground.back()->b = 0;
			_ground.back()->alpha = 1;

			// get user defined values from xml
			double lx, ly, lz, px, py, pz, psi, theta, phi, mass;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->r);
				ele->QueryDoubleAttribute("g", &_ground.back()->g);
				ele->QueryDoubleAttribute("b", &_ground.back()->b);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->alpha);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->x));
				ele->QueryDoubleAttribute("y", &(_ground.back()->y));
				ele->QueryDoubleAttribute("z", &(_ground.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &psi);
				ele->QueryDoubleAttribute("theta", &theta);
				ele->QueryDoubleAttribute("phi", &phi);
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("x", &(_ground.back()->l1));
				ele->QueryDoubleAttribute("y", &(_ground.back()->l2));
				ele->QueryDoubleAttribute("z", &(_ground.back()->l3));
			}

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);
			dRtoQ(R, _ground.back()->q);

			// create body
			dMass m;
			_ground.back()->body = dBodyCreate(_world);
			if (mass == 0) {
				dBodyDisable(_ground.back()->body);
				mass = 1;
			}
			dMassSetBoxTotal(&m, mass, lx, ly, lz);
			dBodySetPosition(_ground.back()->body, px, py, pz);
			dBodySetRotation(_ground.back()->body, R);

			// position geom
			_ground.back()->geom = dCreateBox(_space, lx, ly, lz);
			dGeomSetBody(_ground.back()->geom, _ground.back()->body);
			dGeomSetOffsetPosition(_ground.back()->geom, -m.c[0], -m.c[1], -m.c[2]);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(_ground.back()->body, &m);
		}
		else if ( !strcmp(node->Value(), "cylinder") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->r = 0;
			_ground.back()->g = 0;
			_ground.back()->b = 0;
			_ground.back()->alpha = 1;

			// get user defined values from xml
			double r, l, px, py, pz, psi, theta, phi, mass;
			int axis;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if (node->QueryIntAttribute("axis", &axis)) {
				axis = 1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->r);
				ele->QueryDoubleAttribute("g", &_ground.back()->g);
				ele->QueryDoubleAttribute("b", &_ground.back()->b);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->alpha);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &psi);
				ele->QueryDoubleAttribute("theta", &theta);
				ele->QueryDoubleAttribute("phi", &phi);
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
				ele->QueryDoubleAttribute("length", &l);
			}

			// set rotation of object
			dMatrix3 R, R_x, R_y, R_z, R_xy;
			dRFromAxisAndAngle(R_x, 1, 0, 0, psi);
			dRFromAxisAndAngle(R_y, 0, 1, 0, theta);
			dRFromAxisAndAngle(R_z, 0, 0, 1, phi);
			dMultiply0(R_xy, R_x, R_y, 3, 3, 3);
			dMultiply0(R, R_xy, R_z, 3, 3, 3);

			// create body
			dMass m;
			_ground.back()->body = dBodyCreate(_world);
			if (mass == 0) {
				dBodyDisable(_ground.back()->body);
				mass = 1;
			}
			dMassSetCylinderTotal(&m, mass, axis, r, l);
			dBodySetPosition(_ground.back()->body, px, py, pz);
			dBodySetRotation(_ground.back()->body, R);

			// position geom
			_ground.back()->geom = dCreateCylinder(_space, r, l);
			dGeomSetBody(_ground.back()->geom, _ground.back()->body);
			dGeomSetOffsetPosition(_ground.back()->geom, -m.c[0], -m.c[1], -m.c[2]);
			dMatrix3 R1;
			if (axis == 1) {		dRFromAxisAndAngle(R1, 0, 1, 0, M_PI/2); }
			else if (axis == 2) {	dRFromAxisAndAngle(R1, 1, 0, 0, M_PI/2); }
			else if (axis == 3) {	dRFromAxisAndAngle(R1, 0, 0, 1, 0); }
			dGeomSetOffsetRotation(_ground.back()->geom, R1);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(_ground.back()->body, &m);
		}
		else if ( !strcmp(node->Value(), "sphere") ) {
			// store default variables
			_ground.push_back(new Ground());
			_ground.back()->r = 0;
			_ground.back()->g = 0;
			_ground.back()->b = 0;
			_ground.back()->alpha = 1;

			// get user defined values from xml
			double r, px, py, pz, mass;
			if (node->QueryDoubleAttribute("mass", &mass)) {
				mass = 0.1;
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_ground.back()->r);
				ele->QueryDoubleAttribute("g", &_ground.back()->g);
				ele->QueryDoubleAttribute("b", &_ground.back()->b);
				ele->QueryDoubleAttribute("alpha", &_ground.back()->alpha);
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &px);
				ele->QueryDoubleAttribute("y", &py);
				ele->QueryDoubleAttribute("z", &pz);
			}
			if ( (ele = node->FirstChildElement("size")) ) {
				ele->QueryDoubleAttribute("radius", &r);
			}

			// create body
			dMass m;
			_ground.back()->body = dBodyCreate(_world);
			if (mass == 0) {
				dBodyDisable(_ground.back()->body);
				mass = 1;
			}
			dMassSetSphereTotal(&m, mass, r);
			dBodySetPosition(_ground.back()->body, px, py, pz);

			// position geom
			_ground.back()->geom = dCreateSphere(_space, r);
			dGeomSetBody(_ground.back()->geom, _ground.back()->body);
			dGeomSetOffsetPosition(_ground.back()->geom, -m.c[0], -m.c[1], -m.c[2]);

			// set mass center to (0,0,0) of _bodyID
			dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
			dBodySetMass(_ground.back()->body, &m);
		}

		// go to next node
		node = node->NextSiblingElement();
	}

#ifdef ENABLE_GRAPHICS
	//if (_lib) _graphics->readXML(&doc);
#endif // ENABLE_GRAPHICS

	// get root node of xml file
	node = doc.FirstChildElement("sim")->FirstChildElement();

	// check if individual vs preconfig
	node->QueryIntAttribute("type", &_preconfig);

	// loop over all nodes
	while (node) {
		if (node->ToComment()) {}
		else if ( !strcmp(node->Value(), "mobot") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = MOBOT;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("a1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("a2", &(_xmlbot.back()->angle2));
				ele->QueryDoubleAttribute("a3", &(_xmlbot.back()->angle3));
				ele->QueryDoubleAttribute("a4", &(_xmlbot.back()->angle4));
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else if ( !strcmp(node->Value(), "linkboti") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = LINKBOTI;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("f2", &(_xmlbot.back()->angle2));
				ele->QueryDoubleAttribute("f3", &(_xmlbot.back()->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					_xmlbot.back()->psi = 0;
				else if (o == 2)
					_xmlbot.back()->psi = M_PI/2;
				else if (o == 3)
					_xmlbot.back()->psi = M_PI;
				else if (o == 4)
					_xmlbot.back()->psi = 3*M_PI/2;
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else if ( !strcmp(node->Value(), "linkbotl") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = LINKBOTL;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("f2", &(_xmlbot.back()->angle2));
				ele->QueryDoubleAttribute("f3", &(_xmlbot.back()->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					_xmlbot.back()->psi = 0;
				else if (o == 2)
					_xmlbot.back()->psi = M_PI/2;
				else if (o == 3)
					_xmlbot.back()->psi = M_PI;
				else if (o == 4)
					_xmlbot.back()->psi = 3*M_PI/2;
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else if ( !strcmp(node->Value(), "linkbott") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = LINKBOTT;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("f1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("f2", &(_xmlbot.back()->angle2));
				ele->QueryDoubleAttribute("f3", &(_xmlbot.back()->angle3));
			}
			int o;
			if (!node->QueryIntAttribute("orientation", &o)) {
				if (o == 1)
					_xmlbot.back()->psi = 0;
				else if (o == 2)
					_xmlbot.back()->psi = M_PI/2;
				else if (o == 3)
					_xmlbot.back()->psi = M_PI;
				else if (o == 4)
					_xmlbot.back()->psi = 3*M_PI/2;
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else if ( !strcmp(node->Value(), "nxt") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = NXT;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("w1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("w2", &(_xmlbot.back()->angle2));
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else if ( !strcmp(node->Value(), "cubus") ) {
			_xmlbot.push_back(new XMLRobot());
			_xmlbot.back()->type = CUBUS;
			_xmlbot.back()->connected = 0;
			_xmlbot.back()->x = 0; _xmlbot.back()->y = 0; _xmlbot.back()->z = 0;
			_xmlbot.back()->psi = 0; _xmlbot.back()->theta = 0; _xmlbot.back()->phi = 0;
			_xmlbot.back()->angle1 = 0; _xmlbot.back()->angle2 = 0; _xmlbot.back()->angle3 = 0;
			_xmlbot.back()->angle4 = 0; _xmlbot.back()->angle5 = 0; _xmlbot.back()->angle6 = 0;
			_xmlbot.back()->tracking = tracking;
			node->QueryIntAttribute("id", &(_xmlbot.back()->id));
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &(_xmlbot.back()->x));
				ele->QueryDoubleAttribute("y", &(_xmlbot.back()->y));
				ele->QueryDoubleAttribute("z", &(_xmlbot.back()->z));
			}
			if ( (ele = node->FirstChildElement("rotation")) ) {
				ele->QueryDoubleAttribute("psi", &(_xmlbot.back()->psi));
				ele->QueryDoubleAttribute("theta", &(_xmlbot.back()->theta));
				ele->QueryDoubleAttribute("phi", &(_xmlbot.back()->phi));
			}
			if ( (ele = node->FirstChildElement("joint")) ) {
				ele->QueryDoubleAttribute("a1", &(_xmlbot.back()->angle1));
				ele->QueryDoubleAttribute("a2", &(_xmlbot.back()->angle2));
				ele->QueryDoubleAttribute("a3", &(_xmlbot.back()->angle3));
				ele->QueryDoubleAttribute("a4", &(_xmlbot.back()->angle4));
				ele->QueryDoubleAttribute("a5", &(_xmlbot.back()->angle5));
				ele->QueryDoubleAttribute("a6", &(_xmlbot.back()->angle6));
			}
			if (node->QueryIntAttribute("ground", &(_xmlbot.back()->ground))) {
				_xmlbot.back()->ground = -1;
			}
		}
		else {
			if ( !strcmp(node->Value(), "bigwheel") ) {
				ctype = BIGWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "bridge") ) {
				ctype = BRIDGE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "caster") ) {
				ctype = CASTER;
				cnum = 1;
				node->QueryIntAttribute("custom", &custom);
			}
			else if ( !strcmp(node->Value(), "cube") ) {
				ctype = CUBE;
				cnum = 5;
			}
			else if ( !strcmp(node->Value(), "faceplate") ) {
				ctype = FACEPLATE;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "gripper") ) {
				ctype = GRIPPER;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "l") ) {
				ctype = L;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "omnidrive") ) {
				ctype = OMNIDRIVE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "simple") ) {
				ctype = SIMPLE;
				cnum = 2;
			}
			else if ( !strcmp(node->Value(), "smallwheel") ) {
				ctype = SMALLWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "square") ) {
				ctype = SQUARE;
				cnum = 4;
			}
			else if ( !strcmp(node->Value(), "tank") ) {
				ctype = TANK;
				cnum = 3;
			}
			else if ( !strcmp(node->Value(), "tinywheel") ) {
				ctype = TINYWHEEL;
				cnum = 1;
			}
			else if ( !strcmp(node->Value(), "wheel") ) {
				ctype = WHEEL;
				cnum = 1;
				node->QueryDoubleAttribute("radius", &size);
			}
			rtmp = new int[cnum];
			ftmp = new int[cnum];
			ntmp = new int[cnum];
			atmp = new int[cnum];

			// store connector to temp variables
			int i = 0;
			if (cnum == 1) {
				i = 1;
				ntmp[0] = -1;
				atmp[0] = -1;
				node->QueryIntAttribute("robot", &rtmp[0]);
				node->QueryIntAttribute("face", &ftmp[0]);
			}
			else {
				side = node->FirstChildElement();
				while (side) {
					side->QueryIntAttribute("id", &ntmp[i]);
					side->QueryIntAttribute("robot", &rtmp[i]);
					if (side->QueryIntAttribute("conn", &atmp[i]) == tinyxml2::XML_NO_ATTRIBUTE) {
						atmp[i] = -1;
						side->QueryIntAttribute("face", &ftmp[i]);
					}
					else {
						ftmp[i] = ntmp[i];
						side->QueryIntAttribute("conn", &atmp[i]);
						if (atmp[i] == CASTER)
							side->QueryDoubleAttribute("custom", &size);
						else if (atmp[i] == WHEEL)
							side->QueryDoubleAttribute("radius", &size);
					}
					i++;
					side = side->NextSiblingElement();
				}
			}

			// store connectors to each robot
			for (int j = 0; j < i; j++) {
				for (int k = 0; k < _xmlbot.size(); k++) {
					if (_xmlbot[k]->id == rtmp[j]) {
						_xmlbot[k]->conn.push_back(new XMLConn());
						_xmlbot[k]->conn.back()->robot = rtmp[0];
						_xmlbot[k]->conn.back()->face1 = ftmp[0];
						_xmlbot[k]->conn.back()->conn = atmp[j];
						_xmlbot[k]->conn.back()->face2 = ftmp[j];
						_xmlbot[k]->conn.back()->side = ntmp[j];
						_xmlbot[k]->conn.back()->type = ctype;
						_xmlbot[k]->conn.back()->size = size;
					}
				}
			}

			// delete temporary arrays
			delete [] rtmp;
			delete [] ftmp;
			delete [] ntmp;
			delete [] atmp;
		}
*/
		// debug printing
		/*for (int i = 0; i < _xmlbot.size(); i++) {
			printf("type = %d, id = %d\n", _xmlbot[i]->type, _xmlbot[i]->id);
			printf("x = %lf, y = %lf, z = %lf\n", _xmlbot[i]->x, _xmlbot[i]->y, _xmlbot[i]->z);
			printf("psi = %lf, theta = %lf, phi = %lf\n", _xmlbot[i]->psi, _xmlbot[i]->theta, _xmlbot[i]->phi);
			printf("angle1 = %lf, angle2 = %lf, angle3 = %lf, angle4 = %lf\n", \
				_xmlbot[i]->angle1, _xmlbot[i]->angle2, _xmlbot[i]->angle3, _xmlbot[i]->angle4);
			for (int j = 0; j < _xmlbot[i]->conn.size(); j++) {
				printf("on face %d connect with robot %d on his face %d with type %d from side %d with conn %d\n", \
					_xmlbot[i]->conn[j]->face2, _xmlbot[i]->conn[j]->robot, _xmlbot[i]->conn[j]->face1, \
					_xmlbot[i]->conn[j]->type, _xmlbot[i]->conn[j]->side, _xmlbot[i]->conn[j]->conn);
			}
			printf("\n");
		}*/
/*
		// go to next node
		node = node->NextSiblingElement();
	}

	// success
	return 0;
}*/

/**********************************************************
	Public member functions
 **********************************************************/
int RoboSim::addRobot(Robot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

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
				fprintf(stderr, "Error: Could not find NXT in RoboSim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, _xmlbot[i]->id, _robots.size()-1, this);

	// check if robot is colliding with others
	for (int j = 0; j < _robots.size() - 1; j++) {
		if ( (fabs(_robots[j]->robot->getCenter(0) - _xmlbot[i]->x) < 0.1) && (fabs(_robots[j]->robot->getCenter(1) - _xmlbot[i]->y) < 0.1) ) {
			fprintf(stderr, "Warning: Robot %d and Robot %d are possibly colliding.\n", _robots[j]->robot->getID() + 1, _xmlbot[i]->id + 1);
			fprintf(stderr, "         Please check RoboSim GUI for x and y positions that may be too close.\n");
		}
	}

	// build robot
	robot->build(_xmlbot[i]);

	// robot now connected
	_xmlbot[i]->connected = 1;

#ifdef ENABLE_GRAPHICS
	//if (_lib) _robots.back()->node = _graphics->drawRobot(robot, _xmlbot[i], form, _xmlbot[i]->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::addRobot2(ModularRobot *robot, XMLRobot *bot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// give simulation data to robot
	robot->addToSim(_world, _space, bot->id, _robots.size()-1, this);

	// build
	robot->build(bot);

#ifdef ENABLE_GRAPHICS
	//_robots.back()->node = _graphics->drawRobot(robot, bot, form, bot->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::addRobot2(ModularRobot *robot, XMLRobot *bot, ModularRobot *base) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

	// get form of new robot
	int form = 0;
	robot->getFormFactor(form);

	// give simulation data to robot
	robot->addToSim(_world, _space, bot->id, _robots.size()-1, this);

	// build
	dBodyID body = base->getConnectorBodyID(bot->conn[0]->face1);
	dMatrix3 R;
	double m[3] = {0};
	base->getFaceParams(bot->conn[0]->face1, R, m);
	robot->build(bot, R, m, body, bot->conn[0]);

#ifdef ENABLE_GRAPHICS
	//_robots.back()->node = _graphics->drawRobot(robot, bot, form, bot->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::addRobot(ModularRobot *robot) {
	// lock robot data to insert a new one into simulation
	MUTEX_LOCK(&_robot_mutex);

	// create new robot
	_robots.push_back(new Robots());
	_robots.back()->robot = robot;

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
				fprintf(stderr, "Error: Could not find Mobot in RoboSim GUI.\n");
				break;
			case LINKBOTI:
				fprintf(stderr, "Error: Could not find LinkbotI in RoboSim GUI.\n");
				break;
			case LINKBOTL:
				fprintf(stderr, "Error: Could not find LinkbotL in RoboSim GUI.\n");
				break;
			case LINKBOTT:
				fprintf(stderr, "Error: Could not find LinkbotT in RoboSim GUI.\n");
				break;
			case CUBUS:
				fprintf(stderr, "Error: Could not find CUBUS in RoboSim GUI.\n");
				break;
		}
		if (_preconfig) {
			fprintf(stderr, "       Preconfigured Robot Configuration selected.\n");
			fprintf(stderr, "       Please uncheck if you want to use the Individual Robot List.\n");
		}
		exit(-1);
	}

	// give simulation data to robot
	robot->addToSim(_world, _space, _xmlbot[i]->id, _robots.size()-1, this);

	// find if robot is connected to another one
	int j;
	for (j = 0; j < _xmlbot[i]->conn.size(); j++) {
		if (_xmlbot[i]->conn[j]->robot != _xmlbot[i]->id)
			break;
	}

	// if robot is connected to another one
	if (j != _xmlbot[i]->conn.size()) {
		for (int k = 0; k < _robots.size(); k++) {
			if (_robots[k]->robot->getID() == _xmlbot[i]->conn[j]->robot) {
				ModularRobot *r = dynamic_cast<ModularRobot *>(_robots[k]->robot);
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
		for (int k = 0; k < _robots.size() - 1; k++) {
			if ( (fabs(_robots[k]->robot->getCenter(0) - _xmlbot[i]->x) < 0.1) && (fabs(_robots[k]->robot->getCenter(1) - _xmlbot[i]->y) < 0.1) ) {
				std::cerr << "WARNING: Robot " << _robots[k]->robot->getID() + 1 << "and Robot " << _xmlbot[i]->id + 1 << " are possibly colliding." << std::endl;
				std::cerr << "         Please check RoboSim GUI for x and y positions that may be too close." << std::endl;
			}
		}
		// build
		robot->build(_xmlbot[i]);
	}

	// robot now connected
	_xmlbot[i]->connected = 1;

#ifdef ENABLE_GRAPHICS
	//if (_lib) _robots.back()->node = _graphics->drawRobot(robot, _xmlbot[i], form, _xmlbot[i]->tracking);
#endif // ENABLE_GRAPHICS

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	return 0;
}

int RoboSim::deleteRobot(int loc) {
#ifdef ENABLE_GRAPHICS
	// pause simulation to view results only on first delete
	MUTEX_LOCK(&(_pause_mutex));
	static int paused = 0;
	if (!paused++ && _running) {
		_pause = 1;
		MUTEX_UNLOCK(&(_pause_mutex));

		// get HUD and set message
		//if (_lib) _graphics->getHUDText()->setText("Paused: Press any key to end");

		// sleep until pausing halted by user
		MUTEX_LOCK(&(_pause_mutex));
		while (_pause) {
			MUTEX_UNLOCK(&(_pause_mutex));
#ifdef _WIN32
			Sleep(1);
#else
			usleep(1000);
#endif
			MUTEX_LOCK(&(_pause_mutex));
		}
		MUTEX_UNLOCK(&(_pause_mutex));
	}
	MUTEX_UNLOCK(&(_pause_mutex));
#endif // ENABLE_GRAPHICS

	// lock robot data to delete
	MUTEX_LOCK(&_robot_mutex);

#ifdef ENABLE_GRAPHICS
	//if (_lib) _graphics->stageForDelete(_robots[loc]->node);
#endif // ENABLE_GRAPHICS

	// remove robot
	_robots.erase(_robots.begin()+loc);

	// unlock robot data
	MUTEX_UNLOCK(&_robot_mutex);

	// success
	if (!_robots.size())
		return 0;
	else
		return 1;
}

void RoboSim::done(void) {
	SIGNAL(&_running_cond, &_running_mutex, _running = 0);
}

double RoboSim::getClock(void) {
	MUTEX_LOCK(&_clock_mutex);
	double clock = _clock;
	MUTEX_UNLOCK(&_clock_mutex);
	return clock;
}

int RoboSim::getCOR(double &robot, double &ground) {
	robot = _cor[0];
	ground = _cor[1];

	// success
	return 0;
}

int RoboSim::getMu(double &robot, double &ground) {
	robot = _mu[0];
	ground = _mu[1];

	// success
	return 0;
}

int RoboSim::getPause(void) {
	MUTEX_LOCK(&_pause_mutex);
	int pause = _pause;
	MUTEX_UNLOCK(&_pause_mutex);
	return pause;
}

double RoboSim::getStep(void) {
	MUTEX_LOCK(&_step_mutex);
	double step = _step;
	MUTEX_UNLOCK(&_step_mutex);
	return step;
}

int RoboSim::getUnits(void) {
#ifdef ENABLE_GRAPHICS
	//return _graphics->getUnits();
#endif // ENABLE_GRAPHICS
return 0;
}

int RoboSim::runSimulation(void) {
	MUTEX_LOCK(&_running_mutex);
	while (_running) {
		COND_WAIT(&_running_cond, &_running_mutex);
	}
	MUTEX_UNLOCK(&_running_mutex);

	// success
	return 0;
}

int RoboSim::setCollisions(int mode) {
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

int RoboSim::setCOR(double robot, double ground) {
	_cor[0] = robot;
	_cor[1] = ground;

	// success
	return 0;
}

int RoboSim::setLib(int lib) {
	_lib = lib;

	// success
	return 0;
}

int RoboSim::setMu(double robot, double ground) {
	_mu[0] = robot;
	_mu[1] = ground;

	// success
	return 0;
}

int RoboSim::setPause(int mode) {
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

int RoboSim::xmlNewRobot(XMLRobot *bot) {
	_xmlbot.push_back(bot);
	return 0;
}

/**********************************************************
	Private functions
 **********************************************************/
void* RoboSim::simulation_thread(void *arg) {
	// cast to type sim
	RoboSim *sim = (RoboSim *)arg;

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
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_CREATE(&(sim->_robots[j]->thread), (void* (*)(void *))&Robot::simPreCollisionThreadEntry, (void *)sim->_robots[j]->robot);
			}
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_JOIN(sim->_robots[j]->thread);
			}

			// perform ode update
			dSpaceCollide(sim->_space, sim, &sim->collision);
			dWorldStep(sim->_world, sim->_step);
			MUTEX_LOCK(&(sim->_clock_mutex));
			sim->_clock += sim->_step;
			MUTEX_UNLOCK(&(sim->_clock_mutex));
			dJointGroupEmpty(sim->_group);

			// perform post-collision updates
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_CREATE(&(sim->_robots[j]->thread), (void* (*)(void *))&Robot::simPostCollisionThreadEntry, (void *)sim->_robots[j]->robot);
			}
			for (int j = 0; j < sim->_robots.size(); j++) {
				THREAD_JOIN(sim->_robots[j]->thread);
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

	// cleanup
	delete [] dt;

	// end
	return NULL;
}

void RoboSim::collision(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	RoboSim *ptr = (RoboSim *)data;

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

