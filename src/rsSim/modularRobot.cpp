#include <rsSim/ModularRobot>

using namespace rsSim;

ModularRobot::ModularRobot(void) : rsRobots::Robot(rs::ROBOT), rsSim::Robot(0, 0) {
}

ModularRobot::~ModularRobot(void) {
	// destroy connectors array
	for (int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int ModularRobot::connect(char *name, int pause) {
	// create simulation object if necessary
	//if (!g_sim)
	//	g_sim = new RoboSim(name, pause);

	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// add to simulation
	//g_sim->addRobot(this);

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

ConnectorList& ModularRobot::getConnectorList(void) {
	return _conn;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int ModularRobot::addNeighbor(ModularRobot *robot, int myface, int hisface) {
	_neighbor[myface].robot = robot;
	_neighbor[myface].face = hisface;
}

dBodyID ModularRobot::getConnectorBodyID(int face) {
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i]->face == face) return _conn[i]->body;
	}
	return NULL;
}

int ModularRobot::getNeighborCount(int face, int back) {
	int val = 0;
	if (face != -1) {
		if (_neighbor[face].robot) val += _neighbor[face].robot->getNeighborCount(-1, _neighbor[face].face);
	}
	else {
		for (int i = 0; i < _neighbor.size(); i++)
			if (_neighbor[i].robot) {
				if (i != back)
					val += _neighbor[i].robot->getNeighborCount(-1, _neighbor[i].face);
				else
					val += 1;
			}
	}
	return val;
}

double ModularRobot::getNeighborForce(int face, int dir) {
	return _fb[face]->f1[dir];
}

double ModularRobot::getNeighborTorque(int face, int dir) {
	return _fb[face]->t1[dir];
}

