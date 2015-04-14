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

const rs::Pos ModularRobot::getAttachmentForce(void) {
	return rs::Pos(_fb[_dof].f2[0], _fb[_dof].f2[1], _fb[_dof].f2[2]);
}

const rs::Pos ModularRobot::getAttachmentTorque(void) {
	return rs::Pos(_fb[_dof].t2[0], _fb[_dof].t2[1], _fb[_dof].t2[2]);
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
int ModularRobot::fix_body_to_connector(dBodyID cBody, int face, dJointFeedback *fb) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// attach feedback
	dJointSetFeedback(joint, fb);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int ModularRobot::fix_connector_to_body(int face, dBodyID cBody, dJointFeedback *fb, int conn) {
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

	// attach feedback
	//dJointSetFeedback(joint, fb);		// TODO: causes ode 'CHECK_NOT_LOCKED(geom->parent_space)' segfault

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

