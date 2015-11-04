#include <rsSim/ModularRobot>

using namespace rsSim;

/**********************************************************
	public functions
 **********************************************************/
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
	protected functions
 **********************************************************/
int ModularRobot::fix_body_to_connector(dBodyID cBody, short face) {
	// fixed joint
	dJointID joint = dJointCreateFixed(_world, 0);

	// attach to correct body
	dJointAttach(joint, cBody, this->getBodyID(face));

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int ModularRobot::fix_connector_to_body(short face, dBodyID cBody, short conn) {
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

