#include <rs/Macros>
#include <rsXML/Mindstorms>

using namespace rsXML;

Mindstorms::Mindstorms(int form, bool trace) : rsRobots::Robot(form), rsRobots::Mindstorms(form), rsXML::Robot(trace) { };

void Mindstorms::postProcess(void) {
	// adjust height to be above zero
	if (fabs(_p[2]) < (_wheel_radius - rs::EPSILON)) {
		_p.add(_q.multiply(0, 0, _wheel_radius));
	}
}

