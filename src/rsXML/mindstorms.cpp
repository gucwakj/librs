#include <rs/Macros>
#include <rsXML/Mindstorms>

using namespace rsMindstorms;
using namespace rsXML;

Mindstorms::Mindstorms(int form, bool trace) : rsRobots::Robot(form), rsRobots::Mindstorms(form), rsXML::Robot(trace) {
	_a.allocate(NUM_JOINTS);
	for (unsigned int i = 0; i < _a.size(); i++) {
		_a[i] = 0;
	}
};

void Mindstorms::postProcess(void) {
	// adjust height to be above zero
	double p2;
	_q.multiply(this->tiltForWheels(_wheels[0], _wheels[1], p2));
	_p[2] += p2;
}

