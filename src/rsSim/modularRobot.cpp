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
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i].face == face) return _conn[i].body;
	}
	return NULL;
}

int ModularRobot::getConnectorOrientation(int face) {
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i].face == face) return _conn[i].orientation;
	}
	return -1;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int ModularRobot::add_neighbor(ModularRobot *robot, int myface, int hisface) {
	_neighbor[myface].robot = robot;
	_neighbor[myface].face = hisface;
}

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
	dJointSetFeedback(joint, fb);

	// set joint params
	dJointSetFixed(joint);

	// success
	return 0;
}

int ModularRobot::get_neighbor_count(int face, int back) {
	int val = 0;
	if (face != -1) {
		if (_neighbor[face].robot) val += _neighbor[face].robot->get_neighbor_count(-1, _neighbor[face].face);
	}
	else {
		for (int i = 0; i < _neighbor.size(); i++)
			if (_neighbor[i].robot) {
				if (i != back)
					val += _neighbor[i].robot->get_neighbor_count(-1, _neighbor[i].face);
				else
					val += 1;
			}
	}
	return val;
}

double ModularRobot::get_neighbor_force(int face, int dir) {
	return _fb[face].f1[dir];
}

double ModularRobot::get_neighbor_torque(int face, int dir) {
	return _fb[face].t1[dir];
}

