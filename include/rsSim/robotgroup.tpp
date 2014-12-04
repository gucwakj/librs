template<class T>
int Group<T>::addRobot(T &robot) {
	_robots.push_back(&robot);
	return 0;
}

template<class T>
int Group<T>::addRobots(T robots[], int num) {
	for (int i = 0; i < num; i++) {
		_robots.push_back(&robots[i]);
	}
	return 0;
}

template<class T>
int Group<T>::blinkLED(double delay, int num) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->blinkLED(delay, num);
	}
	return 0;
}

template<class T>
int Group<T>::connect(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->connect();
	}
	return 0;
}

template<class T>
int Group<T>::disconnect(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->disconnect();
	}
	return 0;
}

template<class T>
int Group<T>::driveBackward(double angle) {
	driveBackwardNB(angle);
	return moveWait();
}

template<class T>
int Group<T>::driveBackwardNB(double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveBackwardNB(angle);
	}
	return 0;
}

template<class T>
int Group<T>::driveDistance(double distance, double radius) {
	driveDistanceNB(distance, radius);
	return moveWait();
}

template<class T>
int Group<T>::driveDistanceNB(double distance, double radius) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveDistanceNB(distance, radius);
	}
	return 0;
}

template<class T>
int Group<T>::driveForever(void) {
	driveForeverNB();
	return moveWait();
}

template<class T>
int Group<T>::driveForeverNB(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveForeverNB();
	}
	return 0;
}

template<class T>
int Group<T>::driveForward(double angle) {
	driveForwardNB(angle);
	return moveWait();
}

template<class T>
int Group<T>::driveForwardNB(double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveForwardNB(angle);
	}
	return 0;
}

template<class T>
int Group<T>::driveTime(double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveTimeNB(seconds);
	}
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->holdJoints();
	}
	return 0;
}

template<class T>
int Group<T>::driveTimeNB(double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveTimeNB(seconds);
	}
	return 0;
}

template<class T>
int Group<T>::holdJoint(rs::JointID id) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->holdJoint(id);
	}
	return 0;
}

template<class T>
int Group<T>::holdJoints(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->holdJoints();
	}
	return 0;
}

template<class T>
int Group<T>::holdJointsAtExit(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->holdJointsAtExit();
	}
	return 0;
}

template<class T>
int Group<T>::isMoving(void) {
	for (int i = 0; i < _robots.size(); i++) {
		if(_robots[i]->isMoving()) return 1;
	}
	return 0;
}

template<class T>
int Group<T>::isNotMoving(void) {
	for (int i = 0; i < _robots.size(); i++) {
		if(_robots[i]->isNotMoving()) return 1;
	}
	return 0;
}

template<class T>
int Group<T>::moveForeverNB(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveForeverNB();
	}
	return 0;
}

template<class T>
int Group<T>::moveJoint(rs::JointID id, double angle) {
	moveJointNB(id, angle);
	return moveWait();
}

template<class T>
int Group<T>::moveJointNB(rs::JointID id, double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointNB(id, angle);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointByPowerNB(rs::JointID id, int power) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointByPowerNB(id, power);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointForeverNB(rs::JointID id) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointForeverNB(id);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointTime(rs::JointID id, double seconds) {
	this->moveJointTimeNB(id, seconds);

#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif

	// success
	return 0;
}

template<class T>
int Group<T>::moveJointTimeNB(rs::JointID id, double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointTimeNB(id, seconds);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointTo(rs::JointID id, double angle) {
	moveJointToNB(id, angle);
	return moveWait();
}

template<class T>
int Group<T>::moveJointToNB(rs::JointID id, double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointToNB(id, angle);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointToByTrackPos(rs::JointID id, double angle) {
	moveJointToByTrackPosNB(id, angle);
	return moveJointWait(id);
}

template<class T>
int Group<T>::moveJointToByTrackPosNB(rs::JointID id, double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointToByTrackPosNB(id, angle);
	}
	return 0;
}

template<class T>
int Group<T>::moveJointWait(rs::JointID id) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveJointWait(id);
	}
	return 0;
}

template<class T>
int Group<T>::moveTime(double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveForeverNB();
	}
#ifdef _WIN32
	Sleep(seconds * 1000);
#else
	usleep(seconds * 1000000);
#endif
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->holdJoints();
	}
	return 0;
}

template<class T>
int Group<T>::moveTimeNB(double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveTimeNB(seconds);
	}
	return 0;
}

template<class T>
int Group<T>::moveToZero(void) {
	moveToZeroNB();
	return moveWait();
}

template<class T>
int Group<T>::moveToZeroNB(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToZeroNB();
	}
	return 0;
}

template<class T>
int Group<T>::moveWait(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveWait();
	}
	return 0;
}

template<class T>
int Group<T>::relaxJoint(rs::JointID id) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->relaxJoint(id);
	}
	return 0;
}

template<class T>
int Group<T>::relaxJoints(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->relaxJoints();
	}
	return 0;
}

template<class T>
int Group<T>::resetToZero(void) {
	resetToZeroNB();
	return moveWait();
}

template<class T>
int Group<T>::resetToZeroNB(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->resetToZeroNB();
	}
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequency(int frequency, double time) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setBuzzerFrequency(frequency, time);
	}
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequencyOff(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setBuzzerFrequencyOff();
	}
	return 0;
}

template<class T>
int Group<T>::setBuzzerFrequencyOn(int frequency) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setBuzzerFrequencyOn(frequency);
	}
	return 0;
}

template<class T>
int Group<T>::setLEDColor(char *color) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setLEDColor(color);
	}
	return 0;
}

template<class T>
int Group<T>::setLEDColorRGB(int r, int g, int b) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setLEDColorRGB(r, g, b);
	}
	return 0;
}

template<class T>
int Group<T>::setJointSafetyAngle(double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSafetyAngle(angle);
	}
	return 0;
}

template<class T>
int Group<T>::setJointSafetyAngleTimeout(double seconds) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSafetyAngleTimeout(seconds);
	}
	return 0;
}

template<class T>
int Group<T>::setJointSpeed(rs::JointID id, double speed) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeed(id, speed);
	}
	return 0;
}

template<class T>
int Group<T>::setJointSpeedRatio(rs::JointID id, double ratio) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeedRatio(id, ratio);
	}
	return 0;
}

template<class T>
int Group<T>::setSpeed(double speed, double radius) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setSpeed(speed, radius);
	}
	return 0;
}

template<class T>
int Group<T>::traceOff(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->traceOff();
	}
	return 0;
}

template<class T>
int Group<T>::traceOn(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->traceOn();
	}
	return 0;
}

template<class T>
int Group<T>::turnLeft(double angle, double radius, double trackwidth) {
	this->turnLeftNB(angle, radius, trackwidth);
	return moveWait();
}

template<class T>
int Group<T>::turnLeftNB(double angle, double radius, double trackwidth) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->turnLeftNB(angle, radius, trackwidth);
	}
	return 0;
}

template<class T>
int Group<T>::turnRight(double angle, double radius, double trackwidth) {
	this->turnRightNB(angle, radius, trackwidth);
	return moveWait();
}

template<class T>
int Group<T>::turnRightNB(double angle, double radius, double trackwidth) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->turnRightNB(angle, radius, trackwidth);
	}
	return 0;
}

