#include <rsSim/ModularRobot>
#include <iostream>

using namespace rsSim;

ModularRobot::ModularRobot(void) : rsRobots::Robot(rs::ROBOT) {
}

ModularRobot::~ModularRobot(void) {
std::cerr << "rsSim/~ModularRobot start" << std::endl;
	// destroy connectors array
	_conn.empty();
std::cerr << "rsSim/~ModularRobot end" << std::endl;
}

ConnectorList& ModularRobot::getConnectorList(void) {
	return _conn;
}

dBodyID ModularRobot::getConnectorBodyID(int face) {
	for (unsigned int i = 0; i < _conn.size(); i++) {
		if (_conn[i].face == face) return _conn[i].body;
	}
	return NULL;
}

int ModularRobot::getConnectorOrientation(int face) {
	for (unsigned int i = 0; i < _conn.size(); i++) {
		if (_conn[i].face == face) return _conn[i].orientation;
	}
	return -1;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int ModularRobot::fix_body_to_connector(dBodyID cBody, int face) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int ModularRobot::fix_connector_to_body(int face, dBodyID cBody, int conn) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// connector or body part
	dBodyID body;
	if (conn != -1)
		body = this->getConnectorBodyID(face);
	else
		body = this->getBodyID(face);

	// attach to correct body
	dJointAttach(joint, body, cBody);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

