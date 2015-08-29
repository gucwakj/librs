#include <osg/ComputeBoundsVisitor>
#include <osgDB/FileUtils>
#include <osgFX/Outline>

#include <rs/Enum>
#include <rs/Macros>
#include <rsScene/MouseHandler>
#include <rsScene/Scene>
#include <rsScene/SkyTransform>
#include <rsScene/TextureCallback>

using namespace rsScene;

osg::Node::NodeMask NOT_VISIBLE_MASK = 0x0;
osg::Node::NodeMask RECEIVES_SHADOW_MASK = 0x1;
osg::Node::NodeMask CASTS_SHADOW_MASK = 0x2;
osg::Node::NodeMask IS_PICKABLE_MASK = 0x3;
osg::Node::NodeMask VISIBLE_MASK = 0xffffffff;

Scene::Scene(void) : KeyboardHandler() {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

	// staging area for new insertions
	_staging[0] = new osg::Group;
	_staging[1] = new osg::Group;

	// set default level to load
	_level = -1;

	// set default grid options
	_units = false;			// customary
	_grid.push_back(1);		// 1 inch per tic
	_grid.push_back(12);	// 12 inches per hash
	_grid.push_back(-24);	// min x
	_grid.push_back(24);	// max x
	_grid.push_back(-24);	// min y
	_grid.push_back(24);	// max y
	_grid.push_back(1);		// enabled?
	for (int i = 0; i < 6; i++) {
		if (_units) _grid[i] /= 100;
		else _grid[i] /= 39.37;
	}

	// set thread mutex
	MUTEX_INIT(&_thread_mutex);
	_thread = false;
	_osgThread = 0;

	// set texture path
	_model_path = this->getTexturePath();
	_path.resize(rs::NUM_IMAGES);
	_path[rs::GROUND].append(_model_path).append("background/outdoors/terrain.png");
	_path[rs::FRONT].append(_model_path).append("background/outdoors/sky/front.png");
	_path[rs::LEFTSIDE].append(_model_path).append("background/outdoors/sky/left.png");
	_path[rs::BACK].append(_model_path).append("background/outdoors/sky/back.png");
	_path[rs::RIGHTSIDE].append(_model_path).append("background/outdoors/sky/right.png");
	_path[rs::TOP].append(_model_path).append("background/outdoors/sky/top.png");
	_path[rs::BOTTOM].append(_model_path).append("background/outdoors/sky/bottom.png");

	// flags for graphical output options
	_highlight = false;
	_label = true;
}

Scene::~Scene(void) {
	// stop thread
	if (_osgThread) {
		MUTEX_LOCK(&_thread_mutex);
		_thread = false;
		MUTEX_UNLOCK(&_thread_mutex);
		THREAD_JOIN(_osgThread);
		COND_DESTROY(&_graphics_cond);
		MUTEX_DESTROY(&_graphics_mutex);
	}

	// clean mutexes
	MUTEX_DESTROY(&_thread_mutex);
}

/**********************************************************
	public functions
 **********************************************************/
int Scene::addAndRemoveChildren(void) {
	// add new robots to scene
	while (_staging[0]->getNumChildren()) {
		_scene->addChild(_staging[0]->getChild(0));
		_staging[0]->removeChild(0, 1);
	}

	// share newly created data with other nodes
	osgDB::SharedStateManager *ssm = osgDB::Registry::instance()->getSharedStateManager();
	if (ssm) ssm->share(_root);

	// remove robots from scene
	while (_staging[1]->getNumChildren()) {
		_scene->removeChild(_staging[1]->getChild(0));
		_staging[1]->removeChild(0, 1);
	}
	return 0;
}

void Scene::addChildrenToBackground(void) {
	while (_staging[0]->getNumChildren()) {
		_background->addChild(_staging[0]->getChild(0));
		_staging[0]->removeChild(0, 1);
	}
}

void Scene::addHighlight(int id, bool robot, bool exclusive, const rs::Vec &c) {
	// deselect everything
	if (exclusive) {
		// find nodes of intersection
		osg::Group *test = NULL;
		for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
			test = dynamic_cast<osg::Group *>(_scene->getChild(i));
			// get robot node
			if (test && (!test->getName().compare(0, 5, "robot"))) {
				if (dynamic_cast<osgFX::Outline *>(test->getChild(2)->asTransform()->getChild(0)))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(2)->asTransform()->getChild(0)), c);
			}
			// get obstacle node
			else if (test && !test->getName().compare(0, 8, "obstacle")) {
				if (dynamic_cast<osgFX::Outline *>(test->getChild(0)->asTransform()->getChild(0)))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(0)->asTransform()->getChild(0)), c);
			}
			// get marker node
			else if (test && !test->getName().compare(0, 6, "marker")) {
				if (dynamic_cast<osgFX::Outline *>(test->getChild(0)->asTransform()->getChild(0)))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(0)->asTransform()->getChild(0)), c);
			}
		}
	}

	// set highlight of item
	osg::Group *test = NULL;
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		// get robot node
		if (robot && test && !test->getName().compare(std::string("robot").append(std::to_string(id)))) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (this->intersect_new_item(id, cbbv.getBoundingBox())) {
				this->setHUD(true);
				this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
				this->getHUDText()->setText("Robots are Colliding!");
				this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), rs::Vec(1, 0, 0));
			}
			else {
				this->setHUD(false);
				this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), c);
			}
			break;
		}
		// get obstacle node
		else if (!robot && test && !test->getName().compare(std::string("obstacle").append(std::to_string(id)))) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (this->intersect_new_item(id, cbbv.getBoundingBox())) {
				this->setHUD(true);
				this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
				this->getHUDText()->setText("Objects are Colliding!");
				this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), rs::Vec(1, 0, 0));
			}
			else {
				this->setHUD(false);
				this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), c);
			}
			break;
		}
		// get marker node
		else if (!robot && test && !test->getName().compare(std::string("marker").append(std::to_string(id)))) {
			this->setHUD(false);
			this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), c);
			break;
		}
	}
}

int Scene::deleteMarker(int id) {
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		osg::Group *test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		if (test && !test->getName().compare(std::string("marker").append(std::to_string(id)))) {
			_staging[1]->addChild(test);
			return 0;
		}
	}
	return -1;
}

int Scene::deleteObstacle(int id) {
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		osg::Group *test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		if (test && !test->getName().compare(std::string("obstacle").append(std::to_string(id)))) {
			_staging[1]->addChild(test);
			return 0;
		}
	}
	return -1;
}

int Scene::deleteRobot(int id) {
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		osg::Group *test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		if (test && !test->getName().compare(std::string("robot").append(std::to_string(id)))) {
			_staging[1]->addChild(test);
			return 0;
		}
	}
	return -1;
}

void Scene::drawConnector(rsRobots::ModularRobot *robot, Robot *group, int type, int face, int orientation, double size, int side, int conn) {
	switch (robot->getForm()) {
		case rs::DOF:
			this->draw_robot_dof_conn(dynamic_cast<rsRobots::Dof *>(robot), group, type, face, orientation, size, side, conn);
			break;
		case rs::LINKBOTI: case rs::LINKBOTL: case rs::LINKBOTT:
			this->draw_robot_linkbot_conn(dynamic_cast<rsRobots::Linkbot *>(robot), group, type, face, orientation, size, side, conn);
			break;
	}
}

int Scene::drawMarker(int id, int type, const rs::Pos &p1, const rs::Pos &p2, const rs::Vec &c, int size, std::string s) {
	// create geode
	osg::ref_ptr<osg::Group> marker = new osg::Group();
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	// draw specific marker
	switch (type) {
		case rs::DOT: {
			geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), size/500.0)));
			break;
		}
		case rs::LINE: {
			osg::Geometry *geom = new osg::Geometry();
			osg::Vec3Array *vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]));
			geom->setVertexArray(vert);
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
			osg::LineWidth *width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom);
			geode->getOrCreateStateSet()->setAttributeAndModes(width, osg::StateAttribute::ON);
			break;
		}
		case rs::TEXT: {
			osgText::Text *label = new osgText::Text();
			label->setAlignment(osgText::Text::CENTER_CENTER);
			label->setAxisAlignment(osgText::Text::SCREEN);
			label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			label->setCharacterSize(25*size);
			label->setDrawMode(osgText::Text::TEXT);
			label->setPosition(osg::Vec3(0, 0, 0));
			label->setText(s);
			geode->addDrawable(label);
			break;
		}
	}

	// set rendering properties
	geode->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
	geode->setCullingActive(false);

	// add positioning capability
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	pat->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat->setAttitude(osg::Quat(0, 0, 0, 1));
	pat->addChild(geode.get());
	marker->addChild(pat.get());

	// set user properties of node
	marker->setName(std::string("marker").append(std::to_string(id)));

	// add to scenegraph
	_staging[0]->addChild(marker);

	return 0;
}

Obstacle* Scene::drawObstacle(int id, int type, const rs::Pos &p, const rs::Vec &c, const rs::Vec &l, const rs::Quat &q) {
	// create obstacle objects
	osg::ref_ptr<osg::Group> obstacle = new osg::Group();
	osg::ref_ptr<osg::Geode> body = new osg::Geode;

	switch (type) {
		case rs::BOX:
			body->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0, 0, 0), l[0], l[1], l[2])));
			break;
		case rs::CYLINDER:
			body->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[0], l[1])));
			break;
		case rs::SPHERE:
			body->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), l[0])));
			break;
	}

	// set rendering properties
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	body->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
	body->setCullingActive(false);

	// add positioning capability
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
	pat->addChild(body.get());
	obstacle->addChild(pat.get());

	// set user properties of node
	obstacle->setName(std::string("obstacle").append(std::to_string(id)));

	// add to scenegraph
	_staging[0]->addChild(obstacle);

	// success
	return obstacle;
}

Robot* Scene::drawRobot(rsRobots::Robot *robot, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// create new robot
	osg::ref_ptr<osg::Group> group = new osg::Group();

	switch (robot->getForm()) {
		case rs::DOF:
			this->draw_robot_dof(dynamic_cast<rsRobots::Dof *>(robot), group, p, q, a, c, trace);
			break;
		case rs::LINKBOTI: case rs::LINKBOTL: case rs::LINKBOTT:
			this->draw_robot_linkbot(dynamic_cast<rsRobots::Linkbot *>(robot), group, p, q, a, c, trace);
			break;
		case rs::EV3: case rs::NXT:
			this->draw_robot_mindstorms(dynamic_cast<rsRobots::Mindstorms *>(robot), group, p, q, a, c, trace);
			break;
	}

	// set masks
	//robot->setNodeMask(CASTS_SHADOW_MASK);
	//robot->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	osgText::Text *label = new osgText::Text();
	std::string text;
	if (robot->getName().size())
		text.append(robot->getName());
	else
		text.append("Robot " + std::to_string(robot->getID()+1));
	if (_units)
		text.append("\n\n(" + std::to_string(p[0]*100) + ", " + std::to_string(p[1]*100) + ") [cm]");
	else
		text.append("\n\n(" + std::to_string(p[0]*39.37) + ", " + std::to_string(p[1]*39.37) + ") [in]");
	label->setText(text);
	label->setPosition(osg::Vec3(p[0], p[1], p[2] + (robot->getID() % 2 ? 0.08 : 0) + 0.08));
	osg::Geode *label_geode = new osg::Geode();
	label_geode->addDrawable(label);
	label_geode->setNodeMask(NOT_VISIBLE_MASK);
	label_geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	label_geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	label_geode->getOrCreateStateSet()->setRenderBinDetails(22, "RenderBin");
	label_geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	label->setAlignment(osgText::Text::CENTER_CENTER);
	label->setAxisAlignment(osgText::Text::SCREEN);
	label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	label->setCharacterSize(30);
	label->setColor(osg::Vec4(1, 1, 1, 1));
	label->setBoundingBoxColor(osg::Vec4(0, 0, 0, 0.9));
	label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	label->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX);
	group->insertChild(0, label_geode);

	// draw tracking node
	osg::Geode *trackingGeode = new osg::Geode();
	osg::Geometry *trackingLine = new osg::Geometry();
	osg::Vec3Array *trackingVertices = new osg::Vec3Array();
	trackingVertices->push_back(osg::Vec3(p[0], p[1], 0));
	trackingGeode->setNodeMask(((trace) ? VISIBLE_MASK : NOT_VISIBLE_MASK));
	trackingLine->setVertexArray(trackingVertices);
	trackingLine->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1, 1));
	trackingLine->setDataVariance(osg::Object::DYNAMIC);
	trackingLine->setUseDisplayList(false);
	osg::Vec4Array *colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
	trackingLine->setColorArray(colors);
	trackingLine->setColorBinding(osg::Geometry::BIND_OVERALL);
	osg::Point *point = new osg::Point();
	point->setSize(4.0f);
	trackingGeode->getOrCreateStateSet()->setAttributeAndModes(point, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	trackingGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	trackingGeode->addDrawable(trackingLine);
	group->insertChild(1, trackingGeode);

	// set user properties of node
	label_geode->setName("robotHUD");
	trackingGeode->setName("robotTrace");
	group->setName(std::string("robot").append(std::to_string(robot->getID())));

	// add to scenegraph
	_staging[0]->addChild(group);

	// return robot
	return group;
}

void Scene::drawWheel(rsRobots::Robot *robot, Robot *group, int type, int face) {
	switch (robot->getForm()) {
		case rs::EV3: case rs::NXT:
			this->draw_robot_mindstorms_wheel(dynamic_cast<rsRobots::Mindstorms*>(robot), group, type, face);
			break;
	}
}

osgText::Text* Scene::getHUDText(void) {
	// get text geode
	osg::Geode *geode = NULL;
	for (unsigned int i = 0; i < _root->getNumChildren(); i++) {
		if (!_root->getChild(i)->getName().compare("HUDProjection")) {
			geode = _root->getChild(i)->asGroup()->getChild(0)->asTransform()->getChild(0)->asGeode();
			return dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		}
	}
	// return
	return NULL;
}

std::string Scene::getTexturePath(void) {
	std::string path;
#ifdef _WIN32
	DWORD size = 128;
	HKEY key;
#if defined(_WIN64)
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\Wow6432Node\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#else
	RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\SoftIntegration"), 0, KEY_QUERY_VALUE, &key);
#endif
	char base[128];
	RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)base, &size);
	base[size] = '\0';
	if (base[0] == '\0')
		path = "C:/Ch";
	else
		path = base;
	path += "/package/chrobosim/data/";
#else
	osgDB::setLibraryFilePathList("/home/kgucwa/projects/librs/deps/osg3.2.1/build/lib/");
	path = "/home/kgucwa/projects/librs/resources/";
#endif
	return path;
}

void Scene::setBackgroundImage(int pos, std::string path) {
	if (!path.empty()) _path[pos] = path;
}

void Scene::setFrameRate(int rate) {
	_rate = (rate > 0) ? rate : _rate;
}

void Scene::setGrid(std::vector<double> grid, bool draw) {
	// save new values
	if (grid[0] != -1) _grid[0] = grid[0];
	if (grid[1] != -1) _grid[1] = grid[1];
	if (grid[2] != -1) _grid[2] = grid[2];
	if (grid[3] != -1) _grid[3] = grid[3];
	if (grid[4] != -1) _grid[4] = grid[4];
	if (grid[5] != -1) _grid[5] = grid[5];
	if (grid[6] != -1) _grid[6] = grid[6];

	// draw grid if there is a background on which to draw
	if (_level && draw) {
		// remove old grid
		_background->removeChildren(2, _background->getNumChildren());

		// draw new grid
		this->draw_grid(_grid[0], _grid[1], _grid[2], _grid[3], _grid[4], _grid[5], _grid[6]);
	}
}

void Scene::setHighlight(bool highlight) {
	_highlight = highlight;
}

void Scene::setHUD(bool enable) {
	for (unsigned int i = 0; i < _root->getNumChildren(); i++) {
		if (!_root->getChild(i)->getName().compare("HUDProjection")) {
			if (enable)
				_root->getChild(i)->setNodeMask(VISIBLE_MASK);
			else
				_root->getChild(i)->setNodeMask(NOT_VISIBLE_MASK);
			break;
		}
	}
}

void Scene::setLabel(bool label) {
	_label = label;
}

void Scene::setLevel(int level) {
	// exit if level doesn't change
	if (_level == level) return;

	// set new level
	_level = level;

	// remove background pieces
	if (_background && _background->getNumChildren()) {
		_background->removeChildren(0, _background->getNumChildren());
	}

	// level==NONE, return
	if (_level == rs::Level::None) return;

	// draw new level
	switch (level) {
		case rs::Level::Outdoors:
			this->draw_scene_outdoors();
			break;
		case rs::Level::Board:
			this->draw_scene_board();
			break;
	}
}

void Scene::setMouseHandler(rsScene::MouseHandler *mh) {
	// find and remove old handler
	osgViewer::View::EventHandlers eh = _viewer->getEventHandlers();
	for (osgViewer::View::EventHandlers::iterator it = eh.begin(); it != eh.end(); it++) {
		if (!(*it)->getName().compare(0, 5, "mouse")) {
			_viewer->removeEventHandler((*it));
			break;
		}
	}

	if (mh) {
		// set new handler
		mh->setName("mouse");
		_viewer->addEventHandler(mh);
	}
}

void Scene::setPauseText(int pause) {
	if (pause) {
		this->setHUD(true);
		this->getHUDText()->setText("Paused: Press any key to restart");
	}
	else {
		this->setHUD(false);
		this->getHUDText()->setText("");
	}
}

void Scene::setUnits(bool units) {
	_units = units;
}

int Scene::setupCamera(osg::GraphicsContext *gc, double w, double h) {
	// camera properties
	osg::ref_ptr<osg::Camera> _camera = new osg::Camera();
	_camera->setGraphicsContext(gc);
	_camera->setClearColor(osg::Vec4(0, 0, 0, 1));
	_camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	_camera->setViewport(new osg::Viewport(0, 0, w, h));
	_camera->setDrawBuffer(GL_BACK);
	_camera->setReadBuffer(GL_BACK);
	_camera->setViewMatrixAsLookAt(osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	_camera->setComputeNearFarMode(osgUtil::CullVisitor::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
	_camera->setCullingMode(osgUtil::CullVisitor::NO_CULLING);
	_camera->setNearFarRatio(0.00001);
	_viewer->addSlave(_camera.get());

	// viewer camera properties
	osg::ref_ptr<osgGA::OrbitManipulator> cameraManipulator = new osgGA::OrbitManipulator();
	cameraManipulator->setDistance(0.1);
	cameraManipulator->setAllowThrow(false);
	cameraManipulator->setWheelZoomFactor(0);
	cameraManipulator->setVerticalAxisFixed(true);
	cameraManipulator->setElevation(0.5);
	_viewer->setCameraManipulator(cameraManipulator);
	_viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.6, -0.8, 0.5), osg::Vec3f(0.1, 0.3, 0), osg::Vec3f(0, 0, 1));

	// outlining setup
	osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);
	unsigned int clearMask = _camera->getClearMask();
	_camera->setClearMask(clearMask | GL_STENCIL_BUFFER_BIT);
	_camera->setClearStencil(0);

	// success
	return 0;
}

int Scene::setupScene(double w, double h, bool pause) {
	// create the root node
	_root = new osg::Group;

	// add shadow layer
	_scene = new osgShadow::ShadowedScene;
	_root->addChild(_scene);
	_scene->setReceivesShadowTraversalMask(RECEIVES_SHADOW_MASK);
	_scene->setCastsShadowTraversalMask(CASTS_SHADOW_MASK);
	//osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
	//sm->setTextureSize(osg::Vec2s(1024, 1024));
	//_scene->setShadowTechnique(sm.get());

	// add 'sun' light source
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
	osg::ref_ptr<osg::Light> myLight = new osg::Light;
	myLight->setLightNum(0);
	myLight->setAmbient(osg::Vec4(0.1, 0.1, 0.1, 1));
	myLight->setDiffuse(osg::Vec4(1, 1, 1, 0.1));		// sun color: bright white
	myLight->setDirection(osg::Vec3(1, 0, -1));
	myLight->setPosition(osg::Vec4(0, 0, 1000, 0));
	myLight->setSpecular(osg::Vec4(0.1, 0.1, 0.1, 1));
	ls->setLight(myLight.get());
	ls->setLocalStateSetModes(osg::StateAttribute::ON);
	ls->setStateSetModes(*_root->getOrCreateStateSet(), osg::StateAttribute::ON);
	_root->addChild(ls.get());

	// draw global HUD
	this->draw_global_hud(w, h, pause);

	// draw background pieces for levels
	_background = new osg::Group();
	_scene->addChild(_background);
	this->setLevel(rs::Level::Outdoors);

	// optimize the scene graph, remove redundancies
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_root);

	// event handler
	_viewer->addEventHandler(dynamic_cast<KeyboardHandler*>(this));

	// create and add mouse handler
	osg::ref_ptr<MouseHandler> mh = new MouseHandler(this);
	_viewer->addEventHandler(mh);

	// show scene
	_viewer->setSceneData(_root);

	// success
	return 0;
}

int Scene::setupViewer(osgViewer::Viewer *viewer) {
	// creating the viewer
	if (viewer)
		_viewer = viewer;
	else
		_viewer = new osgViewer::Viewer();

	// set threading model
	_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// statistics event handler
	_viewer->addEventHandler(new osgViewer::StatsHandler);

	// success
	return 0;
}

void Scene::start(int pause) {
	// init graphics variables
	COND_INIT(&_graphics_cond);
	MUTEX_INIT(&_graphics_mutex);
	_graphics = false;

	// create thread
	THREAD_CREATE(&_osgThread, (void* (*)(void *))&Scene::graphics_thread, (void *)this);

	// wait for graphics to be set up
	MUTEX_LOCK(&_graphics_mutex);
	while (!_graphics) {
		COND_WAIT(&_graphics_cond, &_graphics_mutex);
	}
	MUTEX_UNLOCK(&_graphics_mutex);
}

void Scene::toggleHighlight(osg::Group *parent, osg::Node *child, const rs::Vec &c, bool on) {
	if (!_highlight) return;

	if (!parent->getName().compare(0, 5, "robot")) {
		// not highlighted yet, do that now
		if (!(dynamic_cast<osgFX::Outline *>(child))) {
			for (unsigned int i = 2; i < parent->getNumChildren(); i++) {
				osgFX::Outline *outline = new osgFX::Outline();
				outline->setWidth(80);
				outline->setColor(osg::Vec4(c[0], c[1], c[2], 1.0));
				outline->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
				outline->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
				outline->addChild(parent->getChild(i)->asTransform()->getChild(0));
				parent->getChild(i)->asTransform()->replaceChild(parent->getChild(i)->asTransform()->getChild(0), outline);
			}
		}
		// already highlighted, take it away
		else if (!on) {
			for (unsigned int i = 2; i < parent->getNumChildren(); i++) {
				osgFX::Outline *parentOutline = dynamic_cast<osgFX::Outline *>(parent->getChild(i)->asTransform()->getChild(0));
				parent->getChild(i)->asTransform()->replaceChild(parentOutline, parentOutline->getChild(0));
			}
		}
	}
	else if (!parent->getName().compare(0, 8, "obstacle") || !parent->getName().compare(0, 6, "marker")) {
		// not highlighted yet, do that now
		if (!(dynamic_cast<osgFX::Outline *>(child))) {
			osgFX::Outline *outline = new osgFX::Outline();
			outline->setWidth(8);
			outline->setColor(osg::Vec4(c[0], c[1], c[2], 1.0));
			outline->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			outline->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			outline->addChild(parent->getChild(0)->asTransform()->getChild(0));
			parent->getChild(0)->asTransform()->replaceChild(parent->getChild(0)->asTransform()->getChild(0), outline);
		}
		// already highlighted, take it away
		else if (!on) {
			osgFX::Outline *parentOutline = dynamic_cast<osgFX::Outline *>(parent->getChild(0)->asTransform()->getChild(0));
			parent->getChild(0)->asTransform()->replaceChild(parentOutline, parentOutline->getChild(0));
		}
	}
}

void Scene::toggleLabel(osg::Group *parent, osg::Node *child) {
	if (!_label) return;

	if (!parent->getName().compare(0, 5, "robot")) {
		osg::Geode *geode = dynamic_cast<osg::Geode *>(parent->getChild(0));
		geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
	}
	else if (!parent->getName().compare(0, 8, "obstacle")) {
		osg::Geode *geode = dynamic_cast<osg::Geode *>(parent->getChild(0));
		geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
	}
}

/**********************************************************
	private functions
 **********************************************************/
osg::Material* Scene::create_material(osg::Vec4 color) {
	osg::Material *material = new osg::Material();
	material->setAmbient(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setEmission(osg::Material::FRONT, color);
	material->setShininess(osg::Material::FRONT, 2);
	material->setSpecular(osg::Material::FRONT, osg::Vec4(1, 1, 1, 1));
	return material;
}

void Scene::draw_grid(double tics, double hash, double minx, double maxx, double miny, double maxy, double enabled) {
	if ((maxx - minx < -rs::Epsilon) || (maxy - miny < -rs::Epsilon))
		return;

	if ( (int)(enabled) ) {
		// grid lines for each sub-foot
		double minx1 = static_cast<int>(ceil(((minx < -rs::Epsilon) ? 1.001 : 0.999)*minx/tics))*tics;
		double miny1 = static_cast<int>(ceil(((miny < -rs::Epsilon) ? 1.001 : 0.999)*miny/tics))*tics;
		double maxx1 = static_cast<int>(floor(((maxx < -rs::Epsilon) ? 0.999 : 1.001)*maxx/tics))*tics;
		double maxy1 = static_cast<int>(floor(((maxy < -rs::Epsilon) ? 0.999 : 1.001)*maxy/tics))*tics;
		int numx1 = static_cast<int>(ceil(1.001*(maxx1 - minx1)/tics));
		int numy1 = static_cast<int>(ceil(1.001*(maxy1 - miny1)/tics));
		int numVertices = 2*numx1 + 2*numy1;
		osg::Geode *gridGeode = new osg::Geode();
		osg::Geometry *gridLines = new osg::Geometry();
		osg::Vec3 *myCoords = new osg::Vec3[numVertices]();
		// draw x lines
		for (int i = 0, j = 0; i < numx1; i++) {
			myCoords[j++] = osg::Vec3(minx1 + i*tics, miny, 0.0);
			myCoords[j++] = osg::Vec3(minx1 + i*tics, maxy, 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx1; i < numy1; i++) {
			myCoords[j++] = osg::Vec3(minx, miny1 + i*tics, 0.0);
			myCoords[j++] = osg::Vec3(maxx, miny1 + i*tics, 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices = new osg::Vec3Array(numVertices, myCoords);
		gridLines->setVertexArray(vertices);
		gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
		// set color
		osg::Vec4Array *colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1, 1, 1, 1));
		gridLines->setColorArray(colors);
		gridLines->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth = new osg::LineWidth();
		linewidth->setWidth(1.0f);
		gridGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode->getOrCreateStateSet()->setRenderBinDetails(-1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode->addDrawable(gridLines);
		gridGeode->setName("tics");
		_background->addChild(gridGeode);

		// grid lines for each foot
		double minx2 = static_cast<int>(ceil(((minx < -rs::Epsilon) ? 1.01 : 0.99)*minx/hash))*hash;
		double miny2 = static_cast<int>(ceil(((miny < -rs::Epsilon) ? 1.01 : 0.99)*miny/hash))*hash;
		double maxx2 = static_cast<int>(floor(((maxx < -rs::Epsilon) ? 0.99 : 1.01)*maxx/hash))*hash;
		double maxy2 = static_cast<int>(floor(((maxy < -rs::Epsilon) ? 0.99 : 1.01)*maxy/hash))*hash;
		int numx2 = static_cast<int>(ceil(1.01*(maxx2 - minx2)/hash));
		int numy2 = static_cast<int>(ceil(1.01*(maxy2 - miny2)/hash));
		int numVertices2 = 2*numx2 + 2*numy2;
		osg::Geode *gridGeode2 = new osg::Geode();
		osg::Geometry *gridLines2 = new osg::Geometry();
		osg::Vec3 *myCoords2 = new osg::Vec3[numVertices2]();
		// draw x lines
		for (int i = 0, j = 0; i < numx2; i++) {
			myCoords2[j++] = osg::Vec3(minx2 + i*hash, miny, 0.0);
			myCoords2[j++] = osg::Vec3(minx2 + i*hash, maxy, 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx2; i < numy2; i++) {
			myCoords2[j++] = osg::Vec3(minx, miny2 + i*hash, 0.0);
			myCoords2[j++] = osg::Vec3(maxx, miny2 + i*hash, 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices2 = new osg::Vec3Array(numVertices2, myCoords2);
		gridLines2->setVertexArray(vertices2);
		gridLines2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices2));
		// set color
		osg::Vec4Array *colors2 = new osg::Vec4Array;
		colors2->push_back(osg::Vec4(1, 0, 0, 1));
		gridLines2->setColorArray(colors2);
		gridLines2->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth2 = new osg::LineWidth();
		linewidth2->setWidth(2.0f);
		gridGeode2->getOrCreateStateSet()->setAttributeAndModes(linewidth2, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode2->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode2->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode2->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode2->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode2->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode2->addDrawable(gridLines2);
		gridGeode2->setName("hash");
		_background->addChild(gridGeode2);

		// x- and y-axis lines
		osg::Geode *gridGeode3 = new osg::Geode();
		osg::Geometry *gridLines3 = new osg::Geometry();
		osg::Vec3 myCoords3[4];
		if ( fabs(maxx) > fabs(minx) ) {
			if (minx < -rs::Epsilon)
				myCoords3[0] = osg::Vec3(minx, 0, 0);
			else
				myCoords3[0] = osg::Vec3(0, 0, 0);
			myCoords3[1] = osg::Vec3(maxx, 0, 0);
		}
		else {
			if (maxx < -rs::Epsilon)
				myCoords3[1] = osg::Vec3(0, 0, 0);
			else
				myCoords3[1] = osg::Vec3(maxx, 0, 0);
			myCoords3[0] = osg::Vec3(minx, 0, 0);
		}
		if ( fabs(maxy) > fabs(miny) ) {
			if (miny < -rs::Epsilon)
				myCoords3[2] = osg::Vec3(0, miny, 0);
			else
				myCoords3[2] = osg::Vec3(0, 0, 0);
			myCoords3[3] = osg::Vec3(0, maxy, 0);
		}
		else {
			if (maxy < -rs::Epsilon)
				myCoords3[3] = osg::Vec3(0, 0, 0);
			else
				myCoords3[3] = osg::Vec3(0, maxy, 0);
			myCoords3[2] = osg::Vec3(0, miny, 0);
		}
		// add vertices
		osg::Vec3Array *vertices3 = new osg::Vec3Array(4, myCoords3);
		gridLines3->setVertexArray(vertices3);
		gridLines3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4));
		// set color
		osg::Vec4Array *colors3 = new osg::Vec4Array;
		colors3->push_back(osg::Vec4(0, 0, 0, 1));
		gridLines3->setColorArray(colors3);
		gridLines3->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::LineWidth *linewidth3 = new osg::LineWidth();
		linewidth3->setWidth(3.0f);
		gridGeode3->getOrCreateStateSet()->setAttributeAndModes(linewidth3, osg::StateAttribute::ON);
		// set rendering properties
		gridGeode3->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		gridGeode3->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		gridGeode3->getOrCreateStateSet()->setRenderBinDetails(-1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		gridGeode3->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode3->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode3->addDrawable(gridLines3);
		gridGeode3->setName("axes");
		_background->addChild(gridGeode3);

		// x-axis label
		osg::ref_ptr<osg::Billboard> xbillboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> xtext = new osgText::Text();
		xtext->setText("x");
		xtext->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		xtext->setAlignment(osgText::Text::CENTER_BASE_LINE);
		xtext->setRotation(osg::Quat(-1.57, osg::Vec3(0, 0, 1)));
		xtext->setCharacterSize(50);
		xtext->setColor(osg::Vec4(0, 0, 0, 1));
		xtext->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		if ( fabs(maxx) > fabs(minx) ) {
			if (maxx < rs::Epsilon)
				xbillboard->addDrawable(xtext, osg::Vec3d(0.1, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(maxx + 0.1, 0.0, 0.0));
		}
		else {
			if (minx < -rs::Epsilon)
				xbillboard->addDrawable(xtext, osg::Vec3d(maxx + 0.1, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(0.1, 0.0, 0.0));
		}
		xbillboard->setMode(osg::Billboard::AXIAL_ROT);
		xbillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNodeMask(~IS_PICKABLE_MASK);
		xbillboard->setName("xlabel");
		_background->addChild(xbillboard);

		// y-axis label
		osg::ref_ptr<osg::Billboard> ybillboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> ytext = new osgText::Text();
		ytext->setText("y");
		ytext->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		ytext->setAlignment(osgText::Text::CENTER_BASE_LINE);
		ytext->setCharacterSize(50);
		ytext->setColor(osg::Vec4(0, 0, 0, 1));
		ytext->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		if ( fabs(maxy) > fabs(miny) ) {
			if (maxy < rs::Epsilon)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.1, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, maxy + 0.1, 0.0));
		}
		else {
			if (miny < -rs::Epsilon)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, maxy + 0.1, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.1, 0.0));
		}
		ybillboard->setMode(osg::Billboard::AXIAL_ROT);
		ybillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNodeMask(~IS_PICKABLE_MASK);
		ybillboard->setName("xlabel");
		_background->addChild(ybillboard);

		// x grid numbering
		osg::ref_ptr<osg::Billboard> xnum_billboard = new osg::Billboard();
		char text[50];
		osg::ref_ptr<osgText::Text> xzero_text = new osgText::Text();
		xzero_text->setText("0");
		xzero_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		xzero_text->setAlignment(osgText::Text::CENTER_CENTER);
		xzero_text->setCharacterSize(30);
		xzero_text->setColor(osg::Vec4(0, 0, 0, 1));
		xzero_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		xnum_billboard->addDrawable(xzero_text, osg::Vec3d(-0.5*tics, -0.5*tics, 0.0));
		// positive
		if (maxx > rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(1.01*maxx/hash + 1); i++) {
				osg::ref_ptr<osgText::Text> xnumpos_text = new osgText::Text();
				if (_units) sprintf(text, "   %.0lf ", 100*i*hash);
				else sprintf(text, "   %.0lf ", 39.37*i*hash);
				xnumpos_text->setText(text);
				xnumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				xnumpos_text->setAlignment(osgText::Text::CENTER_TOP);
				xnumpos_text->setCharacterSize(30);
				xnumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
				xnumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				xnum_billboard->addDrawable(xnumpos_text, osg::Vec3d(i*hash, 0, 0));
			}
		}
		// negative
		if (minx < -rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(fabs(1.01*minx)/hash + 1); i++) {
				osg::ref_ptr<osgText::Text> xnumneg_text = new osgText::Text();
				if (_units) sprintf(text, "%.0lf    ", -100*i*hash);
				else sprintf(text, "%.0lf    ", -39.37*i*hash);
				xnumneg_text->setText(text);
				xnumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				xnumneg_text->setAlignment(osgText::Text::CENTER_TOP);
				xnumneg_text->setCharacterSize(30);
				xnumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
				xnumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				xnum_billboard->addDrawable(xnumneg_text, osg::Vec3d(-i*hash, 0, 0));
			}
		}
		xnum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		xnum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		xnum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		xnum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		xnum_billboard->setName("xnumbering");
		_background->addChild(xnum_billboard);

		// y grid numbering
		osg::ref_ptr<osg::Billboard> ynum_billboard = new osg::Billboard();
		// positive
		if (maxy > rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(1.01*maxy/hash + 1); i++) {
				osg::ref_ptr<osgText::Text> ynumpos_text = new osgText::Text();
				if (_units) sprintf(text, "    %.0lf", 100*i*hash);
				else sprintf(text, "    %.0lf", 39.37*i*hash);
				ynumpos_text->setText(text);
				ynumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				ynumpos_text->setAlignment(osgText::Text::CENTER_TOP);
				ynumpos_text->setCharacterSize(30);
				ynumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
				ynumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				ynum_billboard->addDrawable(ynumpos_text, osg::Vec3d(0, i*hash, 0));
			}
		}
		// negative
		if (miny < -rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(fabs(1.01*miny)/hash + 1); i++) {
				osg::ref_ptr<osgText::Text> ynumneg_text = new osgText::Text();
				if (_units) sprintf(text, "%.0lf    ", -100*i*hash);
				else sprintf(text, "%.0lf    ", -39.37*i*hash);
				ynumneg_text->setText(text);
				ynumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				ynumneg_text->setAlignment(osgText::Text::CENTER_TOP);
				ynumneg_text->setCharacterSize(30);
				ynumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
				ynumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				ynum_billboard->addDrawable(ynumneg_text, osg::Vec3d(0, -i*hash, 0));
			}
		}
		ynum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		ynum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		ynum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		ynum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		ynum_billboard->setName("ynumbering");
		_background->addChild(ynum_billboard);
	}
}

void Scene::draw_global_hud(double w, double h, bool paused) {
	// init variables
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::Projection> projection = new osg::Projection;
	projection->setName("HUDProjection");
	osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
	osg::ref_ptr<osgText::Text> text = new osgText::Text();
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	double p = 0.1;

	// set projection matrix
	projection->setMatrix(osg::Matrix::ortho2D(0, w, 0, h));
	projection->addChild(transform);

	// set view matrix
	transform->setMatrix(osg::Matrix::identity());
	transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	transform->addChild(geode);

	// set rendering
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	geode->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin");
	geode->addDrawable(text);
	geode->addDrawable(geom);

	// text
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setMaximumWidth(w);
	text->setCharacterSize(20);
	text->setThreadSafeRefUnref(true);
	text->setDataVariance(osg::Object::DYNAMIC);
	if (paused) text->setText(std::string("Paused: Press any key to start"));
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setAlignment(osgText::Text::CENTER_CENTER);
	text->setDrawMode(osgText::Text::TEXT);
	text->setPosition(osg::Vec3(0.5*w, 0.5*p*h, 0));
	text->setColor(osg::Vec4(1, 1, 1, 1));
	text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);

	// background rectangle
	osg::Vec3Array *vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(0, 0, -0.1));
	vertices->push_back(osg::Vec3(w, 0, -0.1));
	vertices->push_back(osg::Vec3(w, p*h, -0.1));
	vertices->push_back(osg::Vec3(0, p*h, -0.1));
	geom->setVertexArray(vertices);
	osg::Vec4Array *colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0, 0, 0, 0.6));
	geom->setColorArray(colors, osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	geom->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add to scene
	_root->addChild(projection);
}

void Scene::draw_robot_dof(rsRobots::Dof *robot, Robot *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// initialize variables
	osg::ref_ptr<osg::Node> body[rsDof::Bodies::Num_Parts];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[rsDof::Bodies::Num_Parts];

	// create transforms
	for (int i = 0; i < rsDof::Bodies::Num_Parts; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		group->addChild(pat[i]);
	}

	// set tracing
	robot->setTrace(trace);

	// draw body
	body[rsDof::Bodies::Body] = osgDB::readNodeFile(_model_path + "dof/body.3ds");
	body[rsDof::Bodies::Body]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	pat[rsDof::Bodies::Body]->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat[rsDof::Bodies::Body]->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));

	// draw 'led'
	osg::Cylinder *cyl = new osg::Cylinder(osg::Vec3d(0, -0.02, 0.0308), 0.01, 0.01);
	osg::ShapeDrawable *led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(c[0], c[1], c[2], 1));
	osg::Geode *bodyled = new osg::Geode();
	bodyled->addDrawable(led);
	bodyled->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	bodyled->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	bodyled->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	bodyled->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	bodyled->setCullingActive(false);

	// draw face
	rs::Quat q1 = robot->getRobotBodyQuaternion(robot->getEnabled(), rs::D2R(a[0]), q);
	rs::Pos p1 = robot->getRobotBodyPosition(robot->getEnabled(), p, q);
	body[rsDof::Bodies::Cap] = osgDB::readNodeFile(_model_path + "dof/cap.3ds");
	body[rsDof::Bodies::Cap]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));
	pat[rsDof::Bodies::Cap]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsDof::Bodies::Cap]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// set rendering
	for (int i = 0; i < rsDof::Bodies::Num_Parts; i++) {
		// set rendering properties
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		body[i]->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		body[i]->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
		body[i]->computeBound();
		body[i]->setCullingActive(false);
		pat[i]->addChild(body[i]);
	}
	// add 'led' as second child of body
	pat[rsDof::Bodies::Body]->addChild(bodyled);
}

void Scene::draw_robot_dof_conn(rsRobots::Dof *robot, Robot *group, int type, int face, int orientation, double size, int side, int conn) {
	// get robot p&q
	osg::PositionAttitudeTransform *pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsDof::Bodies::Body));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();

	// get face p&q
	rs::Quat Q1 = robot->getRobotBodyQuaternion(face, 0, rs::Quat(q[0], q[1], q[2], q[3]));
	rs::Pos P1 = robot->getRobotFacePosition(face, rs::Pos(p[0], p[1], p[2]), rs::Quat(q[0], q[1], q[2], q[3]));
	if (conn == -1) {
		P1 = robot->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = robot->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = robot->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = robot->getConnFaceQuaternion(type, side, orientation, Q1);
		type = conn;
		P1 = robot->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = robot->getConnBodyQuaternion(type, orientation, Q1);
	}

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(P1[0], P1[1], P1[2]));
	transform->setAttitude(osg::Quat(Q1[0], Q1[1], Q1[2], Q1[3]));

	// create node to hold mesh
	osg::ref_ptr<osg::Node> node;
	switch (type) {
		case rsDof::Connectors::El:
			node = osgDB::readNodeFile(_model_path + "dof/el.3ds");
			break;
		case rsDof::Connectors::Foot:
			node = osgDB::readNodeFile(_model_path + "dof/foot.3ds");
			break;
	}
	node->computeBound();
	node->setCullingActive(false);

	// set rendering properties
	node->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.125, 0.369, 0.773, 1)));
	node->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	node->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	node->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	node->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);

	// add body to pat
	transform->addChild(node);

	// set user properties of node
	node->setName("connector");

	// add to scenegraph
	group->addChild(transform);
}

void Scene::draw_robot_linkbot(rsRobots::Linkbot *robot, Robot *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// initialize variables
	osg::ref_ptr<osg::Node> body[rsLinkbot::Bodies::Num_Parts];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[rsLinkbot::Bodies::Num_Parts];

	// create transforms
	for (int i = 0; i < rsLinkbot::Bodies::Num_Parts; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		group->addChild(pat[i]);
	}

	// set tracing
	robot->setTrace(trace);

	// draw body
	body[rsLinkbot::Bodies::Body] = osgDB::readNodeFile(_model_path + "linkbot/body.3ds");
	body[rsLinkbot::Bodies::Body]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	pat[rsLinkbot::Bodies::Body]->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat[rsLinkbot::Bodies::Body]->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));

	// draw 'led'
	osg::Cylinder *cyl = new osg::Cylinder(osg::Vec3d(0, -0.02, 0.0308), 0.01, 0.01);
	osg::ShapeDrawable *led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(c[0], c[1], c[2], 1));
	osg::Geode *bodyled = new osg::Geode();
	bodyled->addDrawable(led);
	bodyled->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	bodyled->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	bodyled->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	bodyled->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	bodyled->setCullingActive(false);

	// draw cap 1
	rs::Quat q1 = robot->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap1, rs::D2R(a[rsLinkbot::Bodies::Joint1]), q);
	rs::Pos p1 = robot->getRobotBodyPosition(rsLinkbot::Bodies::Cap1, p, q);
	body[rsLinkbot::Bodies::Cap1] = osgDB::readNodeFile(_model_path + "linkbot/face_rotate.3ds");
	body[rsLinkbot::Bodies::Cap1]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));
	pat[rsLinkbot::Bodies::Cap1]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsLinkbot::Bodies::Cap1]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// draw cap 2
	q1 = robot->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap2, rs::D2R(a[rsLinkbot::Bodies::Joint2]), q);
	p1 = robot->getRobotBodyPosition(rsLinkbot::Bodies::Cap2, p, q);
	if (robot->getForm() == rs::LINKBOTI) {
		body[rsLinkbot::Bodies::Cap2] = osgDB::readNodeFile(_model_path + "linkbot/face_fixed.3ds");
		body[rsLinkbot::Bodies::Cap2]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	}
	else {
		body[rsLinkbot::Bodies::Cap2] = osgDB::readNodeFile(_model_path + "linkbot/face_rotate.3ds");
		body[rsLinkbot::Bodies::Cap2]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));
	}
	pat[rsLinkbot::Bodies::Cap2]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsLinkbot::Bodies::Cap2]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// draw cap 3
	q1 = robot->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap3, rs::D2R(a[rsLinkbot::Bodies::Joint3]), q);
	p1 = robot->getRobotBodyPosition(rsLinkbot::Bodies::Cap3, p, q);
	if (robot->getForm() == rs::LINKBOTL) {
		body[rsLinkbot::Bodies::Cap3] = osgDB::readNodeFile(_model_path + "linkbot/face_fixed.3ds");
		body[rsLinkbot::Bodies::Cap3]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	}
	else {
		body[rsLinkbot::Bodies::Cap3] = osgDB::readNodeFile(_model_path + "linkbot/face_rotate.3ds");
		body[rsLinkbot::Bodies::Cap3]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));
	}
	pat[rsLinkbot::Bodies::Cap3]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsLinkbot::Bodies::Cap3]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// set rendering
	for (int i = 0; i < rsLinkbot::Bodies::Num_Parts; i++) {
		// set rendering properties
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		body[i]->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		body[i]->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
		body[i]->computeBound();
		body[i]->setCullingActive(false);
		pat[i]->addChild(body[i]);
	}
	// add 'led' as second child of body
	pat[rsLinkbot::Bodies::Body]->addChild(bodyled);
}

void Scene::draw_robot_linkbot_conn(rsRobots::Linkbot *robot, Robot *group, int type, int face, int orientation, double size, int side, int conn) {
	// get robot p&q
	osg::PositionAttitudeTransform *pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsLinkbot::Bodies::Body));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();

	// get face p&q
	rs::Quat Q1 = robot->getRobotBodyQuaternion(face, 0, rs::Quat(q[0], q[1], q[2], q[3]));
	rs::Pos P1 = robot->getRobotFacePosition(face, rs::Pos(p[0], p[1], p[2]), rs::Quat(q[0], q[1], q[2], q[3]));
	if (conn == -1) {
		P1 = robot->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = robot->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = robot->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = robot->getConnFaceQuaternion(type, side, orientation, Q1);
		type = conn;
		P1 = robot->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = robot->getConnBodyQuaternion(type, orientation, Q1);
	}

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(P1[0], P1[1], P1[2]));
	transform->setAttitude(osg::Quat(Q1[0], Q1[1], Q1[2], Q1[3]));

	// create node to hold mesh
	osg::ref_ptr<osg::Node> node;
	switch (type) {
		case rsLinkbot::Connectors::BigWheel:
			node = osgDB::readNodeFile(_model_path + "linkbot/bigwheel.3ds");
			break;
		case rsLinkbot::Connectors::Bridge:
			node = osgDB::readNodeFile(_model_path + "linkbot/bridge.3ds");
			break;
		case rsLinkbot::Connectors::Caster:
			node = osgDB::readNodeFile(_model_path + "linkbot/caster.3ds");
			/*if (size) {
				transform->setScale(osg::Vec3d(1, 1, robot->getCasterScale()));
				osg::ref_ptr<osg::PositionAttitudeTransform> transform2 = new osg::PositionAttitudeTransform();
				transform2->setPosition(osg::Vec3d(0, 0, -0.2));
				transform2->addChild(node);
				transform->addChild(transform2);
			}*/
			break;
		case rsLinkbot::Connectors::Cube:
			node = osgDB::readNodeFile(_model_path + "linkbot/cube.3ds");
			break;
		case rsLinkbot::Connectors::DoubleBridge:
			node = osgDB::readNodeFile(_model_path + "linkbot/doublebridge.3ds");
			break;
		case rsLinkbot::Connectors::Ell:
			node = osgDB::readNodeFile(_model_path + "linkbot/el.3ds");
			break;
		case rsLinkbot::Connectors::Faceplate:
			node = osgDB::readNodeFile(_model_path + "linkbot/faceplate.3ds");
			break;
		case rsLinkbot::Connectors::Foot:
			node = osgDB::readNodeFile(_model_path + "linkbot/foot.3ds");
			break;
		case rsLinkbot::Connectors::Gripper:
			node = osgDB::readNodeFile(_model_path + "linkbot/gripper.3ds");
			break;
		case rsLinkbot::Connectors::Omniplate:
			node = osgDB::readNodeFile(_model_path + "linkbot/omnidrive.3ds");
			break;
		case rsLinkbot::Connectors::Simple:
			node = osgDB::readNodeFile(_model_path + "linkbot/simple.3ds");
			break;
		case rsLinkbot::Connectors::SmallWheel:
			node = osgDB::readNodeFile(_model_path + "linkbot/smallwheel.3ds");
			break;
		case rsLinkbot::Connectors::TinyWheel:
			node = osgDB::readNodeFile(_model_path + "linkbot/tinywheel.3ds");
			break;
		case rsLinkbot::Connectors::Wheel:
			node = osgDB::readNodeFile(_model_path + "linkbot/tinywheel.3ds");
			transform->setScale(osg::Vec3d(1, robot->getWheelRatio(rsLinkbot::Connectors::TinyWheel), robot->getWheelRatio(rsLinkbot::Connectors::TinyWheel)));
			break;
	}
	node->computeBound();
	node->setCullingActive(false);

	// set rendering properties
	node->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.125, 0.369, 0.773, 1)));
	node->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	node->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	node->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	node->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);

	// add body to pat
	transform->addChild(node);

	// set user properties of node
	node->setName("connector");

	// add to scenegraph
	group->addChild(transform);
}

void Scene::draw_robot_mindstorms(rsRobots::Mindstorms *robot, Robot *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// create transform
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	group->addChild(pat.get());

	// set tracing
	robot->setTrace(trace);

	// body
	osg::ref_ptr<osg::Node> body = osgDB::readNodeFile(_model_path + "mindstorms/body.3ds");
	body->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	body->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	body->computeBound();
	body->setCullingActive(false);

	// draw 'led'
	osg::Cylinder *cyl = new osg::Cylinder(osg::Vec3d(0, 0.02, 0), 0.01, 0.01);
	osg::ShapeDrawable *led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(c[0], c[1], c[2], 1));
	osg::Geode *bodyled = new osg::Geode();
	bodyled->addDrawable(led);
	bodyled->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	bodyled->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	bodyled->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	bodyled->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	bodyled->computeBound();
	bodyled->setCullingActive(false);

	// add body to transform
	pat->addChild(body);
	// add 'led' as second child of body
	pat->addChild(bodyled);
	// set position of body
	pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
}

void Scene::draw_robot_mindstorms_wheel(rsRobots::Mindstorms *robot, Robot *group, int type, int face) {
	// get robot p&q
	osg::ref_ptr<osg::PositionAttitudeTransform> pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsMindstorms::Bodies::Body));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();

	// wheel position
	rs::Quat q1 = robot->getRobotBodyQuaternion(face, 0, rs::Quat(q[0], q[1], q[2], q[3]));
	rs::Pos p1 = robot->getRobotBodyPosition(face, rs::Pos(p[0], p[1], p[2]), rs::Quat(q[0], q[1], q[2], q[3]));

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	transform->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// wheel
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(_model_path + "mindstorms/wheel.3ds");
	transform->setScale(osg::Vec3d(1, robot->getWheelRatio(type), robot->getWheelRatio(type)));
	node->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));

	// set rendering properties
	node->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	node->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	node->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	node->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	node->computeBound();
	node->setCullingActive(false);

	// add node to transform
	transform->addChild(node);

	// set user properties of node
	node->setName("wheel");

	// add to scenegraph
	group->addChild(transform);
}

void Scene::draw_scene_outdoors(void) {
	// draw skybox
	this->draw_skybox();

	// square geometry
	osg::Geode *geode = new osg::Geode;
	osg::Geometry *geom = new osg::Geometry;
	geode->addDrawable(geom);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	// extents of geom
	osg::Vec3Array *coords = new osg::Vec3Array;
	coords->push_back(osg::Vec3(-10000, -10000, 0));
	coords->push_back(osg::Vec3( 10000, -10000, 0));
	coords->push_back(osg::Vec3( 10000,  10000, 0));
	coords->push_back(osg::Vec3(-10000,  10000, 0));
	geom->setVertexArray(coords);
	// texture coordinates
	osg::Vec2Array *tcoords = new osg::Vec2Array;
	tcoords->push_back(osg::Vec2(0, 0));
	tcoords->push_back(osg::Vec2(1, 0));
	tcoords->push_back(osg::Vec2(1, 1));
	tcoords->push_back(osg::Vec2(0, 1));
	geom->setTexCoordArray(0, tcoords);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	// texture image
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_path[rs::GROUND]));
	tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL), osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderBinDetails(-1, "RenderBin");
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	// add
	_background->addChild(geode);

	// draw grid
	this->draw_grid(_grid[0], _grid[1], _grid[2], _grid[3], _grid[4], _grid[5], _grid[6]);
}

void Scene::draw_scene_board(void) {
	// square geometry
	osg::Geode *geode = new osg::Geode;
	osg::Geometry *geom = new osg::Geometry;
	geode->addDrawable(geom);
	geom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	// extents of geom
	osg::Vec3Array *coords = new osg::Vec3Array;
	coords->push_back(osg::Vec3(-1.219, -0.610, 0));
	coords->push_back(osg::Vec3( 1.219, -0.610, 0));
	coords->push_back(osg::Vec3( 1.219,  0.610, 0));
	coords->push_back(osg::Vec3(-1.219,  0.610, 0));
	geom->setVertexArray(coords);
	// texture coordinates
	osg::Vec2Array *tcoords = new osg::Vec2Array;
	tcoords->push_back(osg::Vec2(0, 0));
	tcoords->push_back(osg::Vec2(1, 0));
	tcoords->push_back(osg::Vec2(1, 1));
	tcoords->push_back(osg::Vec2(0, 1));
	geom->setTexCoordArray(0, tcoords);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	// texture image
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_path[rs::GROUND]));
	tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL), osg::StateAttribute::ON);
	// add
	_background->addChild(geode);
}

void Scene::draw_skybox(void) {
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	osg::ref_ptr<osg::TexEnv> te = new osg::TexEnv;
	te->setMode(osg::TexEnv::REPLACE);
	stateset->setTextureAttributeAndModes(0, te, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexGen> tg = new osg::TexGen;
	tg->setMode(osg::TexGen::NORMAL_MAP);
	stateset->setTextureAttributeAndModes(0, tg, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexMat> tm = new osg::TexMat;
	stateset->setTextureAttribute(0, tm);
	osg::ref_ptr<osg::TextureCubeMap> skymap = new osg::TextureCubeMap;
	osg::Image *imagePosX = osgDB::readImageFile(_path[rs::RIGHTSIDE]);
	osg::Image *imageNegX = osgDB::readImageFile(_path[rs::LEFTSIDE]);
	osg::Image *imagePosY = osgDB::readImageFile(_path[rs::TOP]);
	osg::Image *imageNegY = osgDB::readImageFile(_path[rs::TOP]);
	osg::Image *imagePosZ = osgDB::readImageFile(_path[rs::FRONT]);
	osg::Image *imageNegZ = osgDB::readImageFile(_path[rs::BACK]);
	if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ) {
		skymap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);
		skymap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
		skymap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		skymap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	}
	stateset->setTextureAttributeAndModes(0, skymap, osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setRange(1.0,1.0);
	stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);
	stateset->setRenderBinDetails(-1, "RenderBin");
	stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::ref_ptr<osg::Drawable> drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),1));
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setCullingActive(false);
	geode->setStateSet(stateset);
	geode->addDrawable(drawable);
	osg::ref_ptr<osg::Transform> transform = new SkyTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);
	osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
	clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TextureCallback(*tm));
	clearNode->addChild(transform);
	clearNode->setNodeMask(~IS_PICKABLE_MASK);
	_background->addChild(clearNode);
}

void* Scene::graphics_thread(void *arg) {
	// cast viewer
	Scene *p = (Scene *)arg;

	// initialize variables
	unsigned int width = 0, height = 0;

	// window interface
	osg::GraphicsContext::WindowingSystemInterface *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi) {
		std::cerr << "osg: cannot create windows." << std::endl;
		return NULL;
	}
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	// window traits
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->width = width / 2;
	traits->height = 3 * width / 8;
	traits->x = width - traits->width - 10;
	traits->y = height - traits->height - 50;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->vsync = false;
	traits->sharedContext = 0;

	// graphics context
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid()) {
		gc->setClearColor(osg::Vec4f(0, 0, 0, 1));
	}
	else {
		std::cerr << "osg: cannot create graphics." << std::endl;
		return NULL;
	}

	// set up viewer
	p->setupViewer(NULL);

	// set up the camera
	p->setupCamera(gc.get(), traits->width, traits->height);

	// set up scene
	p->setupScene(traits->width, traits->height, 1);

	// thread is now running
	MUTEX_INIT(&(p->_thread_mutex));
	p->_thread = true;

	// viewer event handlers
	p->_viewer->addEventHandler(new osgViewer::WindowSizeHandler);

	// signal calling function that setup is done
	COND_ACTION(&(p->_graphics_cond), &(p->_graphics_mutex), p->_graphics = true);

	// run viewer
	MUTEX_LOCK(&(p->_thread_mutex));
	while (p->_thread && !p->_viewer->done()) {
		MUTEX_UNLOCK(&(p->_thread_mutex));

		// start clock for frame rate calculation
		double minFrameTime = (p->_rate > 0.0) ? 1.0 / p->_rate : 0.0;
		osg::Timer_t startFrameTick = osg::Timer::instance()->tick();

		// update thread & scene elements
		p->_viewer->frame();
		p->addAndRemoveChildren();

		// pause for proper frame rate
		osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
		double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
		if (frameTime < minFrameTime) OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0*(minFrameTime - frameTime)));

		MUTEX_LOCK(&(p->_thread_mutex));
	}
	MUTEX_UNLOCK(&(p->_thread_mutex));

	// clean up viewer & root
	p->_viewer->setSceneData(NULL);
#ifdef _WIN32_
	delete p->_viewer;
#endif

	// return
	return arg;
}

bool Scene::intersect_new_item(int id, const osg::BoundingBox &bb) {
	// find nodes of intersection
	osg::Group *test = NULL;
	bool retval = false;
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		// get robot node
		if (test && (!test->getName().compare(0, 5, "robot")) && (test->getName().compare(5, 1, std::to_string(id)))) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (bb.intersects(cbbv.getBoundingBox())) {
				this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), rs::Vec(1, 0, 0), true);
				retval = true;
			}
		}
		// get obstacle node
		if (test && !test->getName().compare(0, 8, "obstacle") && (test->getName().compare(8, 1, std::to_string(id)))) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (bb.intersects(cbbv.getBoundingBox())) {
				this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), rs::Vec(1, 0, 0), true);
				retval = true;
			}
		}
	}
	return retval;
}

