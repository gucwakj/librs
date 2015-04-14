#include <rs/Macros>
#include <rsXML/Linkbot>

using namespace rsXML;

Linkbot::Linkbot(int form, bool trace) : rsRobots::Robot(form), rsXML::Robot(trace) { };

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
		// adjust height to be above zero
		if (fabs(_p[2]) < (_body_radius - EPSILON)) {
			_p.add(_q.multiply(0, 0, _body_height/2));
		}

		// check for wheels
		for (unsigned int i = 0; i < _conn.size(); i++) {
			if (_conn[i]->getConn() == rsLinkbot::BIGWHEEL) {
				_p[2] += (_bigwheel_radius - _body_height/2);
				_wheel_radius = _bigwheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::SMALLWHEEL) {
				_p[2] += (_smallwheel_radius - _body_height/2);
				_wheel_radius = _smallwheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::TINYWHEEL) {
				_p[2] += (_tinywheel_radius - _body_height/2);
				_wheel_radius = _tinywheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::WHEEL) {
				_p[2] += (_conn[i]->getSize() - _body_height/2);
				_wheel_radius = _conn[i]->getSize();
				break;
			}
		}

		// tilt for casters
		for (unsigned int i = 0; i < _conn.size(); i++) {
			if (_conn[i]->getType() == rsLinkbot::CASTER && !static_cast<int>(_conn[i]->getSize()))
				this->setPsi(atan2(_wheel_radius - _smallwheel_radius, 0.08575));
		}
	}
}

