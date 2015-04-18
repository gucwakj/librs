#include <rs/Macros>
#include <rsXML/Mindstorms>

using namespace rsXML;

Mindstorms::Mindstorms(int form, bool trace) : rsRobots::Robot(form), rsXML::Robot(trace) {
	_a.allocate(3);
};

void Mindstorms::postProcess(void) {
	// adjust height to be above zero
	if (fabs(_p[2]) < (_wheel_radius - EPSILON)) {
		_p.add(_q.multiply(0, 0, _wheel_radius));
	}
}

