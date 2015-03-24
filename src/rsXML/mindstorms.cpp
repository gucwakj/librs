#include <rs/Macros>
#include <rsXML/Mindstorms>

using namespace rsXML;

Mindstorms::Mindstorms(int form, bool trace) : rsRobots::Robot(form), rsXML::Robot(trace) {
	_a.add(0);
	_a.add(0);
};

void Mindstorms::postProcess(void) {
	// adjust height to be above zero
	if (fabs(_p[2]) < (_body_height/2 - EPSILON)) {
		_p.add(_q.multiply(0, 0, _body_height/2));
	}
}

