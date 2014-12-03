int CubusGroup::move(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6) {
	moveNB(angle1, angle2, angle3, angle4, angle5, angle6);
	return moveWait();
}

int CubusGroup::moveNB(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveNB(angle1, angle2, angle3, angle4, angle5, angle6);
	}
	return 0;
}

int CubusGroup::moveTo(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6) {
	moveToNB(angle1, angle2, angle3, angle4, angle5, angle6);
	return moveWait();
}

int CubusGroup::moveToNB(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->moveToNB(angle1, angle2, angle3, angle4, angle5, angle6);
	}
	return 0;
}

int CubusGroup::setJointSpeeds(double speed1, double speed2, double speed3, double speed4, double speed5, double speed6) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeeds(speed1, speed2, speed3, speed4, speed5, speed6);
	}
	return 0;
}

int CubusGroup::setJointSpeedRatios(double ratio1, double ratio2, double ratio3, double ratio4, double ratio5, double ratio6) {
	for (int i = 0; i < _robots.size(); i++) {
		_robots[i]->setJointSpeedRatios(ratio1, ratio2, ratio3, ratio4, ratio5, ratio6);
	}
	return 0;
}

