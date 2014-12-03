CMobotGroup::CMobotGroup(void) : Group<CMobot>() {
	_motion = 0;
	_thread = new THREAD_T;
}

CMobotGroup::~CMobotGroup(void) {
	THREAD_CANCEL(*_thread);
}

int CMobotGroup::motionArch(double angle) {
	_d = angle;
	_motion++;
	this->motionArchThread(this);

	// success
	return 0;
}

int CMobotGroup::motionArchNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionArchThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionDistance(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	this->motionDistanceThread(this);

	// success
	return 0;
}

int CMobotGroup::motionDistanceNB(double distance, double radius) {
	_d = distance / radius;
	_motion++;
	THREAD_CREATE(_thread, motionDistanceThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormLeft(int num) {
	_i = num;
	_motion++;
	this->motionInchwormLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormLeftNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionInchwormLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormRight(int num) {
	_i = num;
	_motion++;
	this->motionInchwormRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionInchwormRightNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionInchwormRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionRollBackward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollBackwardThread(this);

	// success
	return 0;
}

int CMobotGroup::motionRollBackwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollBackwardThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionRollForward(double angle) {
	_d = angle;
	_motion++;
	this->motionRollForwardThread(this);

	// success
	return 0;
}

int CMobotGroup::motionRollForwardNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionRollForwardThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionSkinny(double angle) {
	_d = angle;
	_motion++;
	this->motionSkinnyThread(this);

	// success
	return 0;
}

int CMobotGroup::motionSkinnyNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionSkinnyThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionStand(void) {
	_motion++;
	this->motionStandThread(this);

	// success
	return 0;
}

int CMobotGroup::motionStandNB(void) {
	_motion++;
	THREAD_CREATE(_thread, motionStandThread, NULL);

	// success
	return 0;
}

int CMobotGroup::motionTurnLeft(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTurnLeftNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTurnRight(double angle) {
	_d = angle;
	_motion++;
	this->motionTurnRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTurnRightNB(double angle) {
	_d = angle;
	_motion++;
	THREAD_CREATE(_thread, motionTurnRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleRight(int num) {
	_i = num;
	_motion++;
	this->motionTumbleRightThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleRightNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionTumbleRightThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleLeft(int num) {
	_i = num;
	_motion++;
	this->motionTumbleLeftThread(this);

	// success
	return 0;
}

int CMobotGroup::motionTumbleLeftNB(int num) {
	_i = num;
	_motion++;
	THREAD_CREATE(_thread, motionTumbleLeftThread, (void *)this);

	// success
	return 0;
}

int CMobotGroup::motionUnstand(void) {
	_motion++;
	this->motionUnstandThread(this);

	// success
	return 0;
}

int CMobotGroup::motionUnstandNB(void) {
	_motion++;
	THREAD_CREATE(_thread, motionUnstandThread, NULL);

	// success
	return 0;
}

int CMobotGroup::motionWait(void) {
	while (_motion > 0) {
#ifdef _WIN32
		Sleep(200);
#else
		usleep(200000);
#endif
	}

	// success
	return 0;
}

int CMobotGroup::move(double angle1, double angle2, double angle3, double angle4) {
	moveNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveNB(double angle1, double angle2, double angle3, double angle4) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveNB(angle1, angle2, angle3, angle4);
	}
	return 0;
}

int CMobotGroup::moveTo(double angle1, double angle2, double angle3, double angle4) {
	moveToNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveToNB(double angle1, double angle2, double angle3, double angle4) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToNB(angle1, angle2, angle3, angle4);
	}
	return 0;
}

int CMobotGroup::moveToByTrackPos(double angle1, double angle2, double angle3, double angle4) {
	moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	return moveWait();
}

int CMobotGroup::moveToByTrackPosNB(double angle1, double angle2, double angle3, double angle4) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToByTrackPosNB(angle1, angle2, angle3, angle4);
	}
	return 0;
}

int CMobotGroup::setJointSpeeds(double speed1, double speed2, double speed3, double speed4) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeeds(speed1, speed2, speed3, speed4);
	}
	return 0;
}

int CMobotGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4);
	}
	return 0;
}

/**********************************************************
	private functions
 **********************************************************/
void* CMobotGroup::motionArchThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, -cmg->_d/2);
	cmg->moveJointToNB(JOINT3, cmg->_d/2);
	cmg->moveJointWait(JOINT2);
	cmg->moveJointWait(JOINT3);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionDistanceThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(RAD2DEG(cmg->_d), 0, 0, RAD2DEG(cmg->_d));

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionInchwormLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();
	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT2, -50);
		cmg->moveJointTo(JOINT3, 50);
		cmg->moveJointTo(JOINT2, 0);
		cmg->moveJointTo(JOINT3, 0);
	}

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionInchwormRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();
	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT3, 50);
		cmg->moveJointTo(JOINT2, -50);
		cmg->moveJointTo(JOINT3, 0);
		cmg->moveJointTo(JOINT2, 0);
	}

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionRollBackwardThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(-cmg->_d, 0, 0, -cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionRollForwardThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(cmg->_d, 0, 0, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionSkinnyThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveJointToNB(JOINT2, cmg->_d);
	cmg->moveJointToNB(JOINT3, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionStandThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();
	cmg->moveJointTo(JOINT2, -85);
	cmg->moveJointTo(JOINT3, 70);
	cmg->moveWait();
	cmg->moveJointTo(JOINT1, 45);
	cmg->moveJointTo(JOINT2, 20);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTumbleLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();

#ifdef _WIN32
	Sleep(1000);
#else
	usleep(1000000);
#endif

	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT2, DEG2RAD(-85));
		cmg->moveJointTo(JOINT3, DEG2RAD(80));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(80));
		cmg->moveJointTo(JOINT2, DEG2RAD(45));
		cmg->moveJointTo(JOINT3, DEG2RAD(-85));
		cmg->moveJointTo(JOINT2, DEG2RAD(80));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(80));
		if (i != (cmg->_i - 1))
			cmg->moveJointTo(JOINT3, DEG2RAD(45));
	}
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveWait();

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTumbleRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->resetToZero();

#ifdef _WIN32
	Sleep(1000);
#else
	usleep(1000000);
#endif

	for (int i = 0; i < cmg->_i; i++) {
		cmg->moveJointTo(JOINT3, DEG2RAD(85));
		cmg->moveJointTo(JOINT2, DEG2RAD(-80));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(-80));
		cmg->moveJointTo(JOINT3, DEG2RAD(-45));
		cmg->moveJointTo(JOINT2, DEG2RAD(85));
		cmg->moveJointTo(JOINT3, DEG2RAD(-80));
		cmg->moveJointTo(JOINT2, DEG2RAD(0));
		cmg->moveJointTo(JOINT3, DEG2RAD(0));
		cmg->moveJointTo(JOINT2, DEG2RAD(-80));
		if (i != (cmg->_i - 1))
			cmg->moveJointTo(JOINT2, DEG2RAD(-45));
	}
	cmg->moveJointToNB(JOINT3, 0);
	cmg->moveJointToNB(JOINT2, 0);
	cmg->moveWait();

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTurnLeftThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(-cmg->_d, 0, 0, cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionTurnRightThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->move(cmg->_d, 0, 0, -cmg->_d);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

void* CMobotGroup::motionUnstandThread(void* arg) {
	CMobotGroup *cmg = (CMobotGroup *)arg;

	// move
	cmg->moveTo(0, 0, 0, 0);
	cmg->moveJointTo(JOINT3, 45);
	cmg->moveJointTo(JOINT2, -85);
	cmg->moveWait();
	cmg->moveTo(0, 0, 0, 0);
	cmg->moveJointTo(JOINT2, 20);

	// motion is complete
	cmg->_motion--;

	// success
	return NULL;
}

