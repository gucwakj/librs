#include <rsRobots/Mindstorms>

using namespace rsRobots;
using namespace rsMindstorms;

Mindstorms::Mindstorms(short form) : Robot(form) {
	_bigwheel_radius = 0.04100;
	this->setBodyHeight(0.04);
	this->setBodyLength(0.1);
	this->setBodyWidth(0.087319);
	_smallwheel_radius = 0.02800;
	this->setWheelDepth(0.02660);
	this->setWheelRadius(0.02800);
	this->addBodyOffset(rs::Pos(0, 0, 0));		// body
	this->addBodyOffset(rs::Pos(-this->getBodyWidth()/2 - this->getWheelDepth()/2 - 0.002, 0, 0));		// wheel1
	this->addBodyOffset(rs::Pos(this->getBodyWidth()/2 + this->getWheelDepth()/2 + 0.002, 0, 0));		// wheel2
}

/**********************************************************
	public functions
 **********************************************************/
const rs::Quat Mindstorms::getRobotBodyQuaternion(short body, float theta, const rs::Quat &q) {
	// new quaternion
	rs::Quat Q(q);

	// wheel rotation
	Q.multiply(rs::Quat(sin(0.5*theta), 0, 0, cos(0.5*theta)));

	// return offset quaternion
	return Q;
}

float Mindstorms::getWheelRatio(short type) {
	switch (type) {
		case Connectors::Big:
			return _bigwheel_radius/_smallwheel_radius;
		case Connectors::Small:
			return 1.0;
	}
	return 0;
}

const rs::Quat Mindstorms::tiltForWheels(short type1, short type2, float &p2) {
	// set wheel on this face
	this->setWheelLeft(type1);
	this->setWheelRight(type2);

	// tilt
	if (type1 == Connectors::None && type2 == Connectors::None) {
		p2 = 0.024417;
		return rs::Quat(-0.0514625, 0, 0, 0.998675);
	}
	else if (type1 == Connectors::Small && type2 == Connectors::Small) {
		p2 = 0.027943;
		return rs::Quat(-0.022158, 0, 0, 0.999754);
	}
	else if (type1 == Connectors::Small && type2 == Connectors::Big) {
		p2 = 0.035519;
		return rs::Quat(0.027529, -0.053935, 0.033079, 0.997616);
	}
	else if (type1 == Connectors::Big && type2 == Connectors::Small) {
		p2 = 0.035519;
		return rs::Quat(0.027529, 0.053935, -0.033079, 0.997616);
	}
	else if (type1 == Connectors::Big && type2 == Connectors::Big) {
		p2 = 0.040384;
		return rs::Quat(0.048244, 0, 0, 0.998836);
	}

	// return default
	p2 = 0;
	return rs::Quat();
}

