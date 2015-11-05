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
void ModularRobot::fix_body_to_connector(dBodyID cBody, short face) {
	// robot body part
	dBodyID body = this->getBodyID(face);

	// create joint if both bodies exist
	if (body && cBody) {
		// fixed joint
		dJointID joint = dJointCreateFixed(_world, 0);

		// attach to correct body
		dJointAttach(joint, cBody, body);

		// set joint params
		dJointSetFixed(joint);
	}
}

void ModularRobot::fix_connector_to_body(short face, dBodyID cBody, short conn) {
	// connector or body part
	dBodyID body = NULL;
	if (conn != -1) body = this->getConnectorBodyID(face);
	else			body = this->getBodyID(face);

	// create joint if both bodies exist
	if (body && cBody) {
		// fixed joint
		dJointID joint = dJointCreateFixed(_world, 0);

		// attach to correct body
		dJointAttach(joint, body, cBody);

		// set joint params
		dJointSetFixed(joint);
	}
}

