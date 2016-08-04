#include <iostream>
#include <rs/Macros>
#include <rsXML/Dof>

using namespace rsXML;
using namespace rsDof;

Dof::Dof(float scale, bool trace) : rsRobots::Robot(rs::Dof), rsRobots::Dof(-1, scale), rsXML::Robot(trace) {
	_a.allocate(Bodies::Num_Joints);
	for (unsigned int i = 0; i < _a.size(); i++) {
		_a[i] = 0;
	}
}

/**********************************************************
	public functions
 **********************************************************/
void Dof::postProcess(void) {
	// find if i am connected to another robot
	if (_conn[0]->getRobot() != _id) {
		_base = _conn[0];
		_conn.erase(_conn.begin() + 0);
	}

	// reposition robot in space
	if (_base == NULL) {
		// adjust height to be above zero
		if (fabs(_p[2]) < (this->getBodyHeight()/2 - rs::Epsilon)) {
			_p.add(0, 0, this->getBodyHeight()/2);
		}
	}
}

