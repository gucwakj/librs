#include <iostream>
#include <rs/Macros>
#include <rsXML/MiniDof>

using namespace rsXML;
using namespace rsMiniDof;

MiniDof::MiniDof(int joint) : rsRobots::Robot(rs::MiniDof), rsRobots::MiniDof(joint), rsXML::Robot(0) {
	_a.allocate(Bodies::Num_Joints);
	for (unsigned int i = 0; i < _a.size(); i++) {
		_a[i] = 0;
	}
};

/**********************************************************
	public functions
 **********************************************************/
void MiniDof::postProcess(void) {
	// find if i am connected to another robot
	for (unsigned int i = 0; i < _conn.size(); i++) {
		if (_conn[i]->getRobot() != _id) {
			_base = _conn[i];
			_conn.erase(_conn.begin() + i);
			break;
		}
	}

	// reposition robot in space
	if (_base == NULL) {
		// adjust height to be above zero
		if (fabs(_p[2]) < (this->getBodyHeight()/2 - rs::Epsilon)) {
			_p.add(0, 0, this->getBodyHeight()/2);
		}
	}
}

