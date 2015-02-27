#include <rsSim/ModularRobot>

using namespace rsSim;

ModularRobot::ModularRobot(void) : rsRobots::Robot(rs::ROBOT), rsSim::Robot(0, 0) {
}

ModularRobot::~ModularRobot(void) {
	// destroy connectors array
	for (int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int ModularRobot::connect(char *name, int pause) {
	// create simulation object if necessary
	//if (!g_sim)
	//	g_sim = new RoboSim(name, pause);

	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// add to simulation
	//g_sim->addRobot(this);

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

ConnectorList& ModularRobot::getConnectorList(void) {
	return _conn;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
int ModularRobot::addNeighbor(ModularRobot *robot, int myface, int hisface) {
	_neighbor[myface].robot = robot;
	_neighbor[myface].face = hisface;
}

/*int ModularRobot::addSensor(int type, int face) {
	// add new sensor
	_sensor.push_back(new Sensor());
	_sensor.back()->type = type;
	_sensor.back()->space = dSimpleSpaceCreate(0);
	_sensor.back()->body = dBodyCreate(_world);

	// position sensor body
	dMatrix3 R;
	double p[3] = {0};
	double dim[3] = {0.1, 0.01, 0.01};
	this->getFaceParams(face, R, p);
	p[0] += R[0]*dim[0]/2;
	p[1] += R[4]*dim[0]/2;
	p[2] += R[8]*dim[0]/2;
	dBodySetPosition(_sensor.back()->body, p[0], p[1], p[2]);
	dBodySetRotation(_sensor.back()->body, R);
	dBodyDisable(_sensor.back()->body);
	double size[3] = {	fabs(R[0]*dim[0] + R[1]*dim[1] + R[2]*dim[2]),
						fabs(R[4]*dim[0] + R[5]*dim[1] + R[6]*dim[2]),
						fabs(R[8]*dim[0] + R[9]*dim[1] + R[10]*dim[2])};
	_sensor.back()->geom = dCreateBox(_sensor.back()->space, size[0], size[1], size[2]);
	dGeomSetBody(_sensor.back()->geom, _sensor.back()->body);
}*/

/*void ModularRobot::collideSensor(void *data, dGeomID o1, dGeomID o2) {
	// cast void pointer to pointer to class
	ModularRobot *ptr = (ModularRobot *)data;
	char str[300];
	sprintf(str, "id: %d\t", ptr->_id);
	// get bodies of geoms
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2)) return;

	// do not collide with this robot
	if (ptr->_space == dGeomGetSpace(o2)) return;

	// collide with geoms in other robot spaces
	if (dGeomIsSpace(o2)) {
		dSpaceCollide2(o1, o2, ptr, &ptr->collideSensor);
	}
	else {
		dContact contact[8] = {0};
		for (int i = 0; i < dCollide(o1, o2, 8, &contact[0].geom, sizeof(dContact)); i++) {
			sprintf(&str[strlen(str)], "%lf %lf %lf\t", contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
			sprintf(&str[strlen(str)], "%lf %lf %lf\t", contact[i].geom.normal[0], contact[i].geom.normal[1], contact[i].geom.normal[2]);
			sprintf(&str[strlen(str)], "%lf\t", contact[i].geom.depth);
		}
		//printf("%s\n\n", str);
	}
}*/

/*int ModularRobot::drawSensor(int i, osg::Group *robot) {
	// get sensor location
	dQuaternion Q;
	dVector3 dims;
	const double *p = dBodyGetPosition(_sensor[i]->body);
	const double *rot = dBodyGetQuaternion(_sensor[i]->body);
	Q[0] = rot[0]; Q[1] = rot[1]; Q[2] = rot[2]; Q[3] = rot[3];
	dGeomBoxGetLengths(_sensor[i]->geom, dims);

	// draw osg shape for sensor
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(p[0], p[1], p[2]), dims[0], dims[1], dims[2]));
	shape->setColor(osg::Vec4(0, 1, 0, 0.1));
	geode->addDrawable(shape);

	// PAT to transform geom
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	transform->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));

	// set rendering properties
	geode->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	geode->setCullingActive(false);

	// add positioning capability
	transform->addChild(geode);

	// set user properties of node
	geode->setName("sensor");

	// add to scenegraph
	robot->addChild(transform);

	// success
	return 0;
}*/

dBodyID ModularRobot::getConnectorBodyID(int face) {
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i]->face == face) return _conn[i]->body;
	}
	return NULL;
}

int ModularRobot::getNeighborCount(int face, int back) {
	int val = 0;
	if (face != -1) {
		if (_neighbor[face].robot) val += _neighbor[face].robot->getNeighborCount(-1, _neighbor[face].face);
	}
	else {
		for (int i = 0; i < _neighbor.size(); i++)
			if (_neighbor[i].robot) {
				if (i != back)
					val += _neighbor[i].robot->getNeighborCount(-1, _neighbor[i].face);
				else
					val += 1;
			}
	}
	return val;
}

double ModularRobot::getNeighborForce(int face, int dir) {
	return _fb[face]->f1[dir];
}

double ModularRobot::getNeighborTorque(int face, int dir) {
	return _fb[face]->t1[dir];
}

