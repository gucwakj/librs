#include <iostream>
#include <rs/Macros>
#include <rsXML/Robot>

using namespace rsXML;

Robot::Robot(bool trace) : rsRobots::Robot(rs::ROBOT) {
	_a[0] = 0;
	_a[1] = 0;
	_a[2] = 0;
	_a[3] = 0;
	_a[4] = 0;
	_a[5] = 0;
	_base == NULL;
	_connected = 0;
	_ground = -1;
	_id = -1;
	_led[0] = 0;
	_led[1] = 0;
	_led[2] = 0;
	_led[3] = 1;
	_p[0] = 0;
	_p[1] = 0;
	_p[2] = 0;
	_q[0] = 0;
	_q[1] = 0;
	_q[2] = 0;
	_q[3] = 1;
	_trace = trace;
}

Robot::~Robot(void) {
	for (int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int Robot::addConnector(Conn *conn) {
	_conn.push_back(conn);

	// success
	return 0;
}

Conn* Robot::getBaseConnector(void) {
	return _base;
}

ConnectorList& Robot::getConnectorList(void) {
	return _conn;
}

int Robot::getConnect(void) {
	return _connected;
}

int Robot::getGround(void) {
	return _ground;
}

int Robot::getID(void) {
	return _id;
}

double* Robot::getJoints(void) {
	return _a;
}

double* Robot::getLED(void) {
	return _led;
}

std::string Robot::getName(void) {
	return _name;
}

int Robot::getOrientation(void) {
	return _orientation;
}

double* Robot::getPosition(void) {
	return _p;
}

double* Robot::getQuaternion(void) {
	return _q;
}

bool Robot::getTrace(void) {
	return _trace;
}

void Robot::printDebug(void) {
	std::cerr << "XML Robot" << std::endl;
	std::cerr << "form: " << _form << std::endl;
	std::cerr << "  id: " << _id << std::endl;
	std::cerr << "p[0]: " << _p[0] << std::endl;
	std::cerr << "p[1]: " << _p[1] << std::endl;
	std::cerr << "p[2]: " << _p[2] << std::endl;
	std::cerr << "q[0]: " << _q[0] << std::endl;
	std::cerr << "q[1]: " << _q[1] << std::endl;
	std::cerr << "q[2]: " << _q[2] << std::endl;
	std::cerr << "q[3]: " << _q[3] << std::endl;
	std::cerr << "a[0]: " << _a[0] << std::endl;
	std::cerr << "a[1]: " << _a[1] << std::endl;
	std::cerr << "a[2]: " << _a[2] << std::endl;
	std::cerr << "a[3]: " << _a[3] << std::endl;
	std::cerr << "a[4]: " << _a[4] << std::endl;
	std::cerr << "a[5]: " << _a[5] << std::endl;
	for (int i = 0; i < _conn.size(); i++) {
		_conn[i]->printDebug();
	}
}

void Robot::setConnect(int a) {
	_connected = a;
}

void Robot::setGround(int a) {
	_ground = a;
}

void Robot::setID(int a) {
	_id = a;
}

void Robot::setJoints(double a, double b, double c, double d, double e, double f) {
	_a[0] = a;
	_a[1] = b;
	_a[2] = c;
	_a[3] = d;
	_a[4] = e;
	_a[5] = f;
}

void Robot::setLED(double a, double b, double c, double d) {
	_led[0] = a;
	_led[1] = b;
	_led[2] = c;
	_led[3] = d;
}

void Robot::setName(std::string name) {
	_name = name;
}

void Robot::setOrientation(int a) {
	_orientation = a;
}

void Robot::setPsi(double c) {
	// quaternions to multiply
	double q1[4] = {_q[0], _q[1], _q[2], _q[3]};
	double q2[4] = {0, 0, sin(0.5*c), cos(0.5*c)};

	// calculate new quaternion
	this->multiplyQbyQ(q2, q1, _q);
}

void Robot::setPosition(double a, double b, double c) {
	_p[0] = a;
	_p[1] = b;
	_p[2] = c;
}

void Robot::setRotation(double a, double b, double c) {
	double q1[4] = {sin(0.5*a), 0, 0, cos(0.5*a)};
	double q2[4] = {0, sin(0.5*b), 0, cos(0.5*b)};
	double q3[4] = {0, 0, sin(0.5*c), cos(0.5*c)};
	double o1[4];

	this->multiplyQbyQ(q2, q1, o1);
	this->multiplyQbyQ(q3, o1, _q);
}

void Linkbot::postProcess(void) {
	// find if i am connected to another robot
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i]->getRobot() != _id) {
			_base = _conn[i];
			_conn.erase(_conn.begin() + i);
			break;
		}
	}

	// reposition robot in space
	if (_base == NULL) {
		// check for wheels
		for (int i = 0; i < _conn.size(); i++) {
			if (_conn[i]->getConn() == rsLinkbot::BIGWHEEL) {
				_p[2] += (_bigwheel_radius - _body_height/2);
				_radius = _bigwheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::SMALLWHEEL) {
				_p[2] += (_smallwheel_radius - _body_height/2);
				_radius = _smallwheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::TINYWHEEL) {
				_p[2] += (_tinywheel_radius - _body_height/2);
				_radius = _tinywheel_radius;
				break;
			}
			else if (_conn[i]->getConn() == rsLinkbot::WHEEL) {
				_p[2] += (_conn[i]->getSize() - _body_height/2);
				_radius = _conn[i]->getSize();
				break;
			}
		}

		// tilt for casters
		for (int i = 0; i < _conn.size(); i++) {
			if (_conn[i]->getType() == rsLinkbot::CASTER && !static_cast<int>(_conn[i]->getSize()))
				this->setPsi(atan2(_radius - _smallwheel_radius, 0.08575));
		}

		// adjust height to be above zero
		if (fabs(_p[2]) < (_body_radius - EPSILON)) {
			double v[3] = {0, 0, _body_height/2};
			double uv[3] = {_q[1]*v[2]-_q[2]*v[1], _q[2]*v[0]-_q[0]*v[2], _q[0]*v[1]-_q[1]*v[0]};
			double uuv[3] = {2*(_q[1]*uv[2]-_q[2]*uv[1]), 2*(_q[2]*uv[0]-_q[0]*uv[2]), 2*(_q[0]*uv[1]-_q[1]*uv[0])};
			_p[0] += v[0] + 2*_q[3]*uv[0] + uuv[0];
			_p[1] += v[1] + 2*_q[3]*uv[1] + uuv[1];
			_p[2] += v[2] + 2*_q[3]*uv[2] + uuv[2];
		}
	}
}

void Mindstorms::postProcess(void) {
	// adjust height to be above zero
	if (fabs(_p[2]) < (_body_height/2 - EPSILON)) {
		double v[3] = {0, 0, _body_height/2};
		double uv[3] = {_q[1]*v[2]-_q[2]*v[1], _q[2]*v[0]-_q[0]*v[2], _q[0]*v[1]-_q[1]*v[0]};
		double uuv[3] = {2*(_q[1]*uv[2]-_q[2]*uv[1]), 2*(_q[2]*uv[0]-_q[0]*uv[2]), 2*(_q[0]*uv[1]-_q[1]*uv[0])};
		_p[0] += v[0] + 2*_q[3]*uv[0] + uuv[0];
		_p[1] += v[1] + 2*_q[3]*uv[1] + uuv[1];
		_p[2] += v[2] + 2*_q[3]*uv[2] + uuv[2];
	}
}

