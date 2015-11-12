#include <rs/Macros>
#include <rsXML/Linkbot>

using namespace rsXML;
using namespace rsLinkbot;

Linkbot::Linkbot(int form, bool trace) : rsRobots::Robot(form), rsRobots::Linkbot(form), rsXML::Robot(trace) {
	_a.allocate(Bodies::Num_Joints);
	for (unsigned int i = 0; i < _a.size(); i++) {
		_a[i] = 0;
	}
};

/**********************************************************
	public functions
 **********************************************************/
void Linkbot::postProcess(void) {
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
		// tilt for wheels
		float p2;
		_q.multiply(this->tiltForWheels((int)_wheels[0], (int)_wheels[1], p2));
		_p[2] += p2;

		// adjust height to be above zero
		if (fabs(_p[2]) < (this->getBodyHeight()/2 - rs::Epsilon)) {
			_p.add(0, 0, this->getBodyHeight()/2);
		}

		// TODO: needs deletion when tilt takes casters into account
		// tilt for casters
		for (unsigned int i = 0; i < _conn.size(); i++) {
			if (_conn[i]->getType() == Connectors::Caster && !static_cast<int>(_conn[i]->getSize()))
				this->setPsi(atan2(this->getWheelRadius() - this->getSmallWheelRadius(), 0.08575));
		}
	}
}

