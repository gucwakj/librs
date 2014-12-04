int CLinkbotTGroup::accelJointAngleNB(rs::JointID id, double a, double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointAngleNB(id, a, angle);
	}
	return 0;
}

int CLinkbotTGroup::accelJointCycloidalNB(rs::JointID id, double angle, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointCycloidalNB(id, angle, t);
	}
	return 0;
}

int CLinkbotTGroup::accelJointHarmonicNB(rs::JointID id, double angle, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointHarmonicNB(id, angle, t);
	}
	return 0;
}

int CLinkbotTGroup::accelJointSmoothNB(rs::JointID id, double a0, double af, double vmax, double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointSmoothNB(id, a0, af, vmax, angle);
	}
	return 0;
}

int CLinkbotTGroup::accelJointTimeNB(rs::JointID id, double a, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointTimeNB(id, a, t);
	}
	return 0;
}

int CLinkbotTGroup::accelJointToMaxSpeedNB(rs::JointID id, double a) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointToMaxSpeedNB(id, a);
	}
	return 0;
}

int CLinkbotTGroup::accelJointToVelocityNB(rs::JointID id, double a, double v) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->accelJointToVelocityNB(id, a, v);
	}
	return 0;
}

int CLinkbotTGroup::closeGripper(void) {
	for (int i = 0; i < _robots.size()-1; i++) {
		_robots[i]->closeGripperNB();
	}
	_robots.back()->closeGripper();
	return 0;
}

int CLinkbotTGroup::closeGripperNB(void) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->closeGripperNB();
	}
	return 0;
}

int CLinkbotTGroup::driveAccelCycloidalNB(double radius, double d, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelCycloidalNB(radius, d, t);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelDistanceNB(double radius, double a, double d) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelDistanceNB(radius, a, d);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelHarmonicNB(double radius, double d, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelHarmonicNB(radius, d, t);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelSmoothNB(double radius, double a0, double af, double vmax, double d) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelSmoothNB(radius, a0, af, vmax, d);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelTimeNB(double radius, double a, double t) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelTimeNB(radius, a, t);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelToMaxSpeedNB(double radius, double a) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelToMaxSpeedNB(radius, a);
	}
	return 0;
}

int CLinkbotTGroup::driveAccelToVelocityNB(double radius, double a, double v) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->driveAccelToVelocityNB(radius, a, v);
	}
	return 0;
}

int CLinkbotTGroup::move(double angle1, double angle2, double angle3) {
	moveNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveNB(double angle1, double angle2, double angle3) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveNB(angle1, angle2, angle3);
	}
	return 0;
}

int CLinkbotTGroup::moveTo(double angle1, double angle2, double angle3) {
	moveToNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveToNB(double angle1, double angle2, double angle3) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToNB(angle1, angle2, angle3);
	}
	return 0;
}

int CLinkbotTGroup::moveToByTrackPos(double angle1, double angle2, double angle3) {
	moveToByTrackPosNB(angle1, angle2, angle3);
	return moveWait();
}

int CLinkbotTGroup::moveToByTrackPosNB(double angle1, double angle2, double angle3) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToByTrackPosNB(angle1, angle2, angle3);
	}
	return 0;
}

int CLinkbotTGroup::openGripper(double angle) {
	openGripperNB(angle);
	return moveWait();
}

int CLinkbotTGroup::openGripperNB(double angle) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->openGripperNB(angle);
	}
	return 0;
}

int CLinkbotTGroup::setJointSpeeds(double speed1, double speed2, double speed3) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeeds(speed1, speed2, speed3);
	}
	return 0;
}

int CLinkbotTGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3);
	}
	return 0;
}

