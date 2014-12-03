int CNXTGroup::move(double angle1, double angle2) {
	moveNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveNB(double angle1, double angle2) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveNB(angle1, angle2);
	}
	return 0;
}

int CNXTGroup::moveTo(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveToNB(double angle1, double angle2) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToNB(angle1, angle2);
	}
	return 0;
}

int CNXTGroup::moveToByTrackPos(double angle1, double angle2) {
	moveToNB(angle1, angle2);
	return moveWait();
}

int CNXTGroup::moveToByTrackPosNB(double angle1, double angle2) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToByTrackPosNB(angle1, angle2);
	}
	return 0;
}

int CNXTGroup::setJointSpeeds(double speed1, double speed2) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeeds(speed1, speed2);
	}
	return 0;
}

int CNXTGroup::setJointSpeedRatios(double ratio1, double ratio2) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeedRatios(ratio1, ratio2);
	}
	return 0;
}

