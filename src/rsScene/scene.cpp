#include <osgFX/Scribe>

#include <rsScene/mouseHandler.hpp>
#include <rsScene/scene.hpp>
#include <rsScene/skyTransform.hpp>
#include <rsScene/textureCallback.hpp>

using namespace rsScene;

osg::Node::NodeMask NOT_VISIBLE_MASK = 0x0;
osg::Node::NodeMask RECEIVES_SHADOW_MASK = 0x1;
osg::Node::NodeMask CASTS_SHADOW_MASK = 0x2;
osg::Node::NodeMask IS_PICKABLE_MASK = 0x3;
osg::Node::NodeMask VISIBLE_MASK = 0xffffffff;

Scene::Scene(void) : keyboardHandler() {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

	// staging area for new insertions
	_staging = new osg::Group;

	// id of robot for deleting
	_deleting = -1;

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
		if (_units)
			_grid[i] /= 100;
		else
			_grid[i] /= 39.37;
	}

	// set thread mutex
	MUTEX_INIT(&_thread_mutex);
	_thread = false;

	// set texture path
	_tex_path = this->getTexturePath();

	// flags for graphical output options
	_highlight = false;
	_label = true;
}

Scene::~Scene(void) {
	std::cerr << "deleting Scene" << std::endl;
	// stop thread
	MUTEX_LOCK(&_thread_mutex);
	_thread = false;
	MUTEX_UNLOCK(&_thread_mutex);
	THREAD_JOIN(_osgThread);

	// clean mutexes
	MUTEX_DESTROY(&_thread_mutex);
}

/**********************************************************
	public functions
 **********************************************************/
int Scene::addChild(void) {
	if (_staging->getNumChildren()) {
		_scene->addChild(_staging->getChild(0));
		_staging->removeChild(0, 1);
		return _scene->getNumChildren() - 1;
	}
	return -1;
}

void Scene::addHighlight(int id, bool exclusive) {
	// deselect everything
	if (exclusive) {
		// find nodes of intersection
		osg::Group *test = NULL;
		for (int i = 6; i < _scene->getNumChildren(); i++) {
			test = dynamic_cast<osg::Group *>(_scene->getChild(i));
			// get robot node
			if (test && (test->getName() == "robot" || test->getName() == "ground")) {
				if (dynamic_cast<osgFX::Scribe *>(test->getChild(2)->asTransform()->getChild(0)))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(2)->asTransform()->getChild(0)));
			}
		}
	}

	// set highlight of item
	osg::Group *parent = _scene->getChild(id)->asGroup();
	osg::Node *child = parent->getChild(2)->asTransform()->getChild(0);
	if (parent && child) {
		this->toggleHighlight(parent, child);
	}
}

int Scene::deleteChild(int id) {
	_scene->removeChild(_scene->getChild(id));
	return _scene->getNumChildren();
}

void Scene::drawConnector(rsRobots::ModularRobot *robot, Robot *group, int type, int face, double size, int side, int conn) {
	switch (robot->getForm()) {
		case rs::CUBUS:
			//this->drawCubus(dynamic_cast<rsRobots::Cubus*>(robot), p, q, trace, rgb);
			break;
		case rs::LINKBOTI:
			this->draw_linkbot_conn(dynamic_cast<rsRobots::LinkbotI*>(robot), group, type, face, size, side, conn);
			break;
		case rs::LINKBOTL:
			this->draw_linkbot_conn(dynamic_cast<rsRobots::LinkbotL*>(robot), group, type, face, size, side, conn);
			break;
		case rs::LINKBOTT:
			this->draw_linkbot_conn(dynamic_cast<rsRobots::LinkbotT*>(robot), group, type, face, size, side, conn);
			break;
		case rs::MOBOT:
			//this->draw(dynamic_cast<rsRobots::CMobot*>(robot), p, q, trace, rgb);
			break;
	}
}

Ground* Scene::drawGround(int type, const double *p, const double *c, const double *l, const double *q) {
	// create ground objects
	osg::ref_ptr<osg::Group> ground = new osg::Group();
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> shape;

	switch (type) {
		case rs::BOX:
			shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0, 0, 0), l[0], l[1], l[2]));
			break;
		case rs::CYLINDER:
			shape = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[0], l[1]));
			break;
		case rs::SPHERE:
			shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), l[0]));
			break;
	}
	shape->setColor(osg::Vec4(c[0], c[1], c[2], c[3]));
	body->addDrawable(shape);

	// set rendering properties
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	body->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	body->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	body->setCullingActive(false);

	// add positioning capability
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
	pat->addChild(body.get());
	ground->addChild(pat.get());

	// set user properties of node
	ground->setName("ground");

	// optimize object
	osgUtil::Optimizer optimizer;
	optimizer.optimize(ground);

	// add to scenegraph
	_staging->addChild(ground);

	// success
	return ground;
}

int Scene::drawMarker(int type, const double *p1, const double *p2, const double *c, int size, std::string s) {
	// create geode
	osg::Geode *geode = new osg::Geode();

	// draw specific marker
	switch (type) {
		case rs::DOT: {
			osg::Sphere *sphere = new osg::Sphere(osg::Vec3d(p1[0], p1[1], p1[2]), size/500.0);
			osg::ShapeDrawable *pointDrawable = new osg::ShapeDrawable(sphere);
			pointDrawable->setColor(osg::Vec4(c[0], c[1], c[2], c[3]));
			geode->addDrawable(pointDrawable);
			break;
		}
		case rs::LINE: {
			osg::Geometry *geom = new osg::Geometry();
			osg::Vec3Array *vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(p1[0], p1[1], p1[2]));
			vert->push_back(osg::Vec3(p2[0], p2[1], p2[2]));
			geom->setVertexArray(vert);
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
			osg::Vec4Array *colors = new osg::Vec4Array;
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors);
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::LineWidth *width = new osg::LineWidth();
			width->setWidth(size*3.0f);
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
			label->setCharacterSize(25);
			label->setColor(osg::Vec4(c[0], c[1], c[2], c[3]));
			label->setDrawMode(osgText::Text::TEXT);
			label->setPosition(osg::Vec3(p1[0], p1[1], p1[2]));
			label->setText(s);
			geode->addDrawable(label);
			break;
		}
	}

	// set rendering properties
	geode->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// optimize object
	osgUtil::Optimizer optimizer;
	optimizer.optimize(geode);

	// add to scenegraph
	_staging->addChild(geode);

	return 0;
}

Robot* Scene::drawRobot(rsRobots::Robot *robot, int form, const double *p, const double *q, bool trace) {
	// create new robot
	osg::Group *group = new osg::Group();
	double rgb[4] = {0};

	switch (form) {
		case rs::CUBUS:
			//this->drawCubus(dynamic_cast<rsRobots::Cubus*>(robot), p, q, trace, rgb);
			break;
		case rs::LINKBOTI:
			this->draw_linkbot(dynamic_cast<rsRobots::LinkbotI*>(robot), group, p, q, trace, rgb);
			break;
		case rs::LINKBOTL:
			this->draw_linkbot(dynamic_cast<rsRobots::LinkbotL*>(robot), group, p, q, trace, rgb);
			break;
		case rs::LINKBOTT:
			this->draw_linkbot(dynamic_cast<rsRobots::LinkbotT*>(robot), group, p, q, trace, rgb);
			break;
		case rs::MOBOT:
			//this->draw(dynamic_cast<rsRobots::CMobot*>(robot), p, q, trace, rgb);
			break;
		case rs::NXT:
			//this->draw(dynamic_cast<rsRobots::CNXT*>(robot), p, q, trace, rgb);
			break;
	}

	// set masks
	//robot->setNodeMask(CASTS_SHADOW_MASK);
	//robot->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	osgText::Text *label = new osgText::Text();
	char text[50];
	sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", robot->getID() + 1, p[0]*100, p[1]*100);
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
	trackingGeode->setNodeMask((trace) ? VISIBLE_MASK : NOT_VISIBLE_MASK);
	trackingLine->setVertexArray(trackingVertices);
	trackingLine->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1, 1));
	trackingLine->setDataVariance(osg::Object::DYNAMIC);
	trackingLine->setUseDisplayList(false);
	osg::Vec4Array *colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(rgb[0], rgb[1], rgb[2], 1));
	trackingLine->setColorArray(colors);
	trackingLine->setColorBinding(osg::Geometry::BIND_OVERALL);
	osg::Point *point = new osg::Point();
	point->setSize(4.0f);
	trackingGeode->getOrCreateStateSet()->setAttributeAndModes(point, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	trackingGeode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	trackingGeode->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	trackingGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	trackingGeode->addDrawable(trackingLine);
	//group->insertChild(1, trackingGeode);
	group->insertChild(1, label_geode);

	// set user properties of node
	group->setName("robot");

	// optimize robot
	osgUtil::Optimizer optimizer;
	optimizer.optimize(group);

	// add to scenegraph
	_staging->addChild(group);

	// return robot
	return group;
}

osgText::Text* Scene::getHUDText(void) {
	// get text geode
	osg::Geode *geode = _scene->getParent(0)->getChild(1)->asGroup()->getChild(0)->asTransform()->getChild(0)->asGeode();
	// return text
	return dynamic_cast<osgText::Text *>(geode->getDrawable(0));
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
	char path[128];
	RegQueryValueEx(key, TEXT("CHHOME"), NULL, NULL, (LPBYTE)path, &size);
	path[size] = '\0';
	if (path[0] == '\0')
		path = "C:/Ch";
	else
		path = path;
	path += "/package/chrobosim/data/";
#else
	path = "/usr/local/ch/package/chrobosim/data/";
#endif
	return path;
}

void Scene::setDelete(int id) {
	_deleting = id;
}

void Scene::setGrid(bool units, std::vector<double> grid) {
	_units = units;

	_grid[0] = grid[0];
	_grid[1] = grid[1];
	_grid[2] = grid[2];
	_grid[3] = grid[3];
	_grid[4] = grid[4];
	_grid[5] = grid[5];
	_grid[6] = grid[6];
}

void Scene::setHighlight(bool highlight) {
	_highlight = highlight;
}

void Scene::setLabel(bool label) {
	_label = label;
}

void Scene::setPauseText(int pause) {
	if (pause)
		this->getHUDText()->setText("Paused: Press any key to restart");
	else
		this->getHUDText()->setText("");
}

int Scene::setupCamera(osg::GraphicsContext *gc, double w, double h) {
	// camera properties
	_camera = new osg::Camera;
	_camera->setGraphicsContext(gc);
	_camera->setViewport(new osg::Viewport(0, 0, w, h));
	_camera->setDrawBuffer(GL_BACK);
	_camera->setReadBuffer(GL_BACK);
	_camera->setViewMatrixAsLookAt(osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 0), osg::Vec3f(0, 0, 1));
	_camera->setComputeNearFarMode(osgUtil::CullVisitor::COMPUTE_NEAR_FAR_USING_PRIMITIVES);
	_camera->setCullingMode(osgUtil::CullVisitor::NO_CULLING);
	_camera->setNearFarRatio(0.00001);
	_viewer->addSlave(_camera);

	// viewer camera properties
	osg::ref_ptr<osgGA::OrbitManipulator> cameraManipulator = new osgGA::OrbitManipulator();
	cameraManipulator->setDistance(0.1);
	cameraManipulator->setAllowThrow(false);
	cameraManipulator->setWheelZoomFactor(0);
	cameraManipulator->setVerticalAxisFixed(true);
	cameraManipulator->setElevation(0.5);
	_viewer->setCameraManipulator(cameraManipulator);
	_viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.6, -0.8, 0.5), osg::Vec3f(0.1, 0.3, 0), osg::Vec3f(0, 0, 1));

	// event handler
	_viewer->addEventHandler(new osgGA::StateSetManipulator(_camera->getOrCreateStateSet()));

	// success
	return 0;
}

int Scene::setupScene(double w, double h) {
	// Creating the root node
	_root = new osg::Group;

	// add shadows
	_scene = new osgShadow::ShadowedScene;
	_root->addChild(_scene);
	_scene->setReceivesShadowTraversalMask(RECEIVES_SHADOW_MASK);
	_scene->setCastsShadowTraversalMask(CASTS_SHADOW_MASK);
	//osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
	//sm->setTextureSize(osg::Vec2s(1024, 1024));
	//shadowedScene->setShadowTechnique(sm.get());

	// add light source
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0.5, 1.0, 1.0, 0.0));
	ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1.0));
	ls->getLight()->setAmbient(osg::Vec4(1, 1, 1, 1.0));
	ls->getLight()->setConstantAttenuation(0.05);
	ls->getLight()->setQuadraticAttenuation(0.05);
	_scene->addChild(ls.get());

	// load terrain node
	osg::ref_ptr<osg::Depth> t_depth = new osg::Depth;
	t_depth->setFunction(osg::Depth::LEQUAL);
	t_depth->setRange(1.0, 1.0);
	osg::ref_ptr<osg::StateSet> t_stateset = new osg::StateSet();
	t_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	t_stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	t_stateset->setAttributeAndModes(t_depth, osg::StateAttribute::ON);
	t_stateset->setRenderBinDetails(-1, "RenderBin");
	t_stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::ref_ptr<osg::Node> t_geode = osgDB::readNodeFile(_tex_path + "ground/terrain.3ds");
	t_geode->setCullingActive(false);
	t_geode->setStateSet(t_stateset);
	osg::ref_ptr<osg::PositionAttitudeTransform> t_transform = new osg::PositionAttitudeTransform();
	t_transform->setScale(osg::Vec3d(2, 2, 0.001));
	t_transform->setCullingActive(false);
	t_transform->addChild(t_geode);
	osg::ref_ptr<osgUtil::LineSegmentIntersector> r_segment = new osgUtil::LineSegmentIntersector(osg::Vec3d(0, 0, 999), osg::Vec3d(0, 0, -999));
	osgUtil::IntersectionVisitor r_visitor(r_segment);
	t_transform->accept(r_visitor);
	osgUtil::LineSegmentIntersector::Intersection r_hits = r_segment->getFirstIntersection();
	osg::Vec3d r_pos = r_hits.getWorldIntersectPoint();
	t_transform->setPosition(osg::Vec3d(r_pos[0], r_pos[1], -r_pos[2]));
	//t_transform->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
	t_transform->setNodeMask(~IS_PICKABLE_MASK);
	_scene->addChild(t_transform);

	// set up HUD
	osg::ref_ptr<osg::Geode> HUDGeode = new osg::Geode();
	osg::ref_ptr<osgText::Text> textHUD = new osgText::Text();
	osg::ref_ptr<osg::Projection> HUDProjectionMatrix = new osg::Projection;
	osg::ref_ptr<osg::MatrixTransform> HUDModelViewMatrix = new osg::MatrixTransform;
	osg::ref_ptr<osg::StateSet> HUDStateSet = new osg::StateSet();
	HUDProjectionMatrix->setMatrix(osg::Matrix::ortho2D(0, w, 0, h));
	HUDProjectionMatrix->addChild(HUDModelViewMatrix);
	HUDModelViewMatrix->setMatrix(osg::Matrix::identity());
	HUDModelViewMatrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	HUDModelViewMatrix->addChild(HUDGeode);
	HUDGeode->setStateSet(HUDStateSet);
	HUDStateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
	HUDStateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	HUDStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	HUDStateSet->setRenderBinDetails(11, "RenderBin");
	HUDGeode->addDrawable(textHUD);
	textHUD->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	textHUD->setMaximumWidth(w);
	textHUD->setCharacterSize(15);
	if (_deleting) textHUD->setText("Paused: Press any key to start");
	textHUD->setAxisAlignment(osgText::Text::SCREEN);
	textHUD->setAlignment(osgText::Text::CENTER_CENTER);
	textHUD->setPosition(osg::Vec3(w/2, 50, -1.5));
	textHUD->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	textHUD->setBackdropColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	textHUD->setBackdropOffset(0.1);
	_root->addChild(HUDProjectionMatrix);

	// grid
	if ( (int)(_grid[6]) ) {
		// grid lines for each sub-foot
		double minx = (int)((_grid[2]*1.01)/_grid[0])*_grid[0];
		double miny = (int)((_grid[4]*1.01)/_grid[0])*_grid[0];
		int numx = (int)((_grid[3] - minx)/_grid[0]+1);
		int numy = (int)((_grid[5] - miny)/_grid[0]+1);
		int numVertices = 2*numx + 2*numy;
		osg::Geode *gridGeode = new osg::Geode();
		osg::Geometry *gridLines = new osg::Geometry();
		osg::Vec3 *myCoords = new osg::Vec3[numVertices]();
		// draw x lines
		for (int i = 0, j = 0; i < numx; i++) {
			myCoords[j++] = osg::Vec3(minx + i*_grid[0], _grid[4], 0.0);
			myCoords[j++] = osg::Vec3(minx + i*_grid[0], _grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx; i < numy; i++) {
			myCoords[j++] = osg::Vec3(_grid[2], miny + i*_grid[0], 0.0);
			myCoords[j++] = osg::Vec3(_grid[3], miny + i*_grid[0], 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices = new osg::Vec3Array(numVertices, myCoords);
		gridLines->setVertexArray(vertices);
		gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
		// set color
		osg::Vec4Array *colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );	// white
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
		_scene->addChild(gridGeode);

		// grid lines for each foot
		double minx2 = (int)((_grid[2]*1.01)/_grid[1])*_grid[1];
		double miny2 = (int)((_grid[4]*1.01)/_grid[1])*_grid[1];
		int numx2 = (int)((_grid[3] - minx2)/_grid[1]+1);
		int numy2 = (int)((_grid[5] - miny2)/_grid[1]+1);
		int numVertices2 = 2*numx2 + 2*numy2;
		osg::Geode *gridGeode2 = new osg::Geode();
		osg::Geometry *gridLines2 = new osg::Geometry();
		osg::Vec3 *myCoords2 = new osg::Vec3[numVertices2]();
		// draw x lines
		for (int i = 0, j = 0; i < numx2; i++) {
			myCoords2[j++] = osg::Vec3(minx2 + i*_grid[1], _grid[4], 0.0);
			myCoords2[j++] = osg::Vec3(minx2 + i*_grid[1], _grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx2; i < numy2; i++) {
			myCoords2[j++] = osg::Vec3(_grid[2], miny2 + i*_grid[1], 0.0);
			myCoords2[j++] = osg::Vec3(_grid[3], miny2 + i*_grid[1], 0.0);
		}
		// add vertices
		osg::Vec3Array *vertices2 = new osg::Vec3Array(numVertices2, myCoords2);
		gridLines2->setVertexArray(vertices2);
		gridLines2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices2));
		// set color
		osg::Vec4Array *colors2 = new osg::Vec4Array;
		colors2->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );	// red
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
		_scene->addChild(gridGeode2);

		// x- and y-axis lines
		osg::Geode *gridGeode3 = new osg::Geode();
		osg::Geometry *gridLines3 = new osg::Geometry();
		osg::Vec3 myCoords3[4];
		if ( fabs(_grid[3]) > fabs(_grid[2]) ) {
			if (_grid[2] < -EPSILON)
				myCoords3[0] = osg::Vec3(-_grid[3], 0, 0);
			else
				myCoords3[0] = osg::Vec3(0, 0, 0);
			myCoords3[1] = osg::Vec3(_grid[3], 0, 0);
		}
		else {
			if (_grid[3] < -EPSILON)
				myCoords3[1] = osg::Vec3(0, 0, 0);
			else
				myCoords3[1] = osg::Vec3(_grid[3], 0, 0);
			myCoords3[0] = osg::Vec3(_grid[2], 0, 0);
		}
		if ( fabs(_grid[5]) > fabs(_grid[4]) ) {
			if (_grid[4] < -EPSILON)
				myCoords3[2] = osg::Vec3(-_grid[5], 0, 0);
			else
				myCoords3[2] = osg::Vec3(0, 0, 0);
			myCoords3[3] = osg::Vec3(0, _grid[5], 0);
		}
		else {
			if (_grid[5] < -EPSILON)
				myCoords3[3] = osg::Vec3(0, 0, 0);
			else
				myCoords3[3] = osg::Vec3(0, _grid[5], 0);
			myCoords3[2] = osg::Vec3(0, _grid[4], 0);
		}
		// add vertices
		osg::Vec3Array *vertices3 = new osg::Vec3Array(4, myCoords3);
		gridLines3->setVertexArray(vertices3);
		gridLines3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4));
		// set color
		osg::Vec4Array *colors3 = new osg::Vec4Array;
		colors3->push_back(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
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
		_scene->addChild(gridGeode3);

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
		if ( fabs(_grid[3]) > fabs(_grid[2]) ) {
			if (_grid[3] < EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(0.05, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(_grid[3] + 0.05, 0.0, 0.0));
		}
		else {
			if (_grid[2] < -EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(_grid[3] + 0.05, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(0.05, 0.0, 0.0));
		}
		xbillboard->setMode(osg::Billboard::AXIAL_ROT);
		xbillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNodeMask(~IS_PICKABLE_MASK);
		_root->addChild(xbillboard);

		// y-axis label
		osg::ref_ptr<osg::Billboard> ybillboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> ytext = new osgText::Text();
		ytext->setText("y");
		ytext->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		ytext->setAlignment(osgText::Text::CENTER_BASE_LINE);
		ytext->setCharacterSize(50);
		ytext->setColor(osg::Vec4(0, 0, 0, 1));
		ytext->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		if ( fabs(_grid[5]) > fabs(_grid[4]) ) {
			if (_grid[5] < EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.05, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, _grid[5] + 0.05, 0.0));
		}
		else {
			if (_grid[4] < -EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, _grid[5] + 0.05, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.05, 0.0));
		}
		ybillboard->setMode(osg::Billboard::AXIAL_ROT);
		ybillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNodeMask(~IS_PICKABLE_MASK);
		_root->addChild(ybillboard);

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
		xnum_billboard->addDrawable(xzero_text, osg::Vec3d(-0.5*_grid[0], -0.5*_grid[0], 0.0));
		// positive
		for (int i = 1; i < (int)(_grid[3]/_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumpos_text = new osgText::Text();
			if (_units) sprintf(text, "   %.0lf", 100*i*_grid[1]);
			else sprintf(text, "   %.0lf", 39.37*i*_grid[1]);
			xnumpos_text->setText(text);
			xnumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumpos_text->setCharacterSize(30);
			xnumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumpos_text, osg::Vec3d(i*_grid[1], -0.5*_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(_grid[2])/_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumneg_text = new osgText::Text();
			if (_units) sprintf(text, "%.0lf   ", -100*i*_grid[1]);
			else sprintf(text, "%.0lf   ", -39.37*i*_grid[1]);
			xnumneg_text->setText(text);
			xnumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumneg_text->setCharacterSize(30);
			xnumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumneg_text, osg::Vec3d(-i*_grid[1], -0.5*_grid[0], 0.0));
		}
		xnum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		xnum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		xnum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		xnum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		xnum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		_root->addChild(xnum_billboard);

		// y grid numbering
		osg::ref_ptr<osg::Billboard> ynum_billboard = new osg::Billboard();
		// positive
		for (int i = 1; i < (int)(_grid[5]/_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumpos_text = new osgText::Text();
			if (_units) sprintf(text, "   %.0lf", 100*i*_grid[1]);
			else sprintf(text, "   %.0lf", 39.37*i*_grid[1]);
			ynumpos_text->setText(text);
			ynumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumpos_text->setCharacterSize(30);
			ynumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumpos_text, osg::Vec3d(0, i*_grid[1] + 0.5*_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(_grid[4])/_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumneg_text = new osgText::Text();
			if (_units) sprintf(text, "%.0lf   ", -100*i*_grid[1]);
			else sprintf(text, "%.0lf   ", -39.37*i*_grid[1]);
			ynumneg_text->setText(text);
			ynumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumneg_text->setCharacterSize(30);
			ynumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumneg_text, osg::Vec3d(0, -i*_grid[1] - 0.5*_grid[0], 0.0));
		}
		ynum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		ynum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
		ynum_billboard->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		ynum_billboard->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		ynum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		_root->addChild(ynum_billboard);
	}

	// skybox
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
	osg::Image* imagePosX = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_right.png");
	osg::Image* imageNegX = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_left.png");
	osg::Image* imagePosY = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_top.png");
	osg::Image* imageNegY = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_top.png");
	osg::Image* imagePosZ = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_front.png");
	osg::Image* imageNegZ = osgDB::readImageFile(_tex_path + "ground/checkered/checkered_back.png");
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
	osg::ref_ptr<osg::Transform> transform = new skyTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);
	osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
	clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new textureCallback(*tm));
	clearNode->addChild(transform);
	clearNode->setNodeMask(~IS_PICKABLE_MASK);
	_root->addChild(clearNode);

	// drawing objects
	/*for (int i = 0; i < _drawings.size(); i++) {
		drawMarker(_drawings[i]);
	}*/

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_root);

	// event handler
	_viewer->addEventHandler(dynamic_cast<keyboardHandler*>(this));
	_viewer->addEventHandler(new mouseHandler(this));

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
		_viewer = new osgViewer::Viewer;

	// set threading model
	_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// event handler
	_viewer->addEventHandler(new osgViewer::StatsHandler);

	// success
	return 0;
}

void Scene::start(int pause) {
	// initialize variables
	unsigned int width, height;

	// window interface
	osg::GraphicsContext::WindowingSystemInterface *wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi) {
		osg::notify(osg::NOTICE) << "osg: cannot create windows." << std::endl;
		return;
	}
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	// window traits
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->width = width/2;
	traits->height = 3*width/8;
	traits->x = width - traits->width - 10;
	traits->y = height - traits->height - 50;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->vsync = false;
	traits->sharedContext = 0;

	// graphics context
	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid()) {
		gc->setClearColor(osg::Vec4f(0.f,0.f,0.f,0.f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else {
		osg::notify(osg::NOTICE) << "osg: cannot create graphics." << std::endl;
		return;
	}

	// set up viewer
	this->setupViewer(NULL);

	// set up the camera
	this->setupCamera(gc.get(), traits->width, traits->height);

	// set up scene
	_deleting = pause;
	this->setupScene(traits->width, traits->height);
	_deleting = 0;

    // thread is now running
	MUTEX_INIT(&_thread_mutex);
	_thread = true;

	// create graphics thread
	THREAD_CREATE(&_osgThread, (void* (*)(void *))&Scene::graphics_thread, (void *)this);
}

void Scene::toggleHighlight(osg::Group *parent, osg::Node *child) {
	if (!_highlight) return;

	if (parent->getName() == "robot") {
		// not highlighted yet, do that now
		if (!(dynamic_cast<osgFX::Scribe *>(child))) {
			for (int i = 2; i < parent->getNumChildren(); i++) {
				osgFX::Scribe *scribe = new osgFX::Scribe();
				scribe->setWireframeColor(osg::Vec4(1, 1, 0, 0.1));
				scribe->setWireframeLineWidth(1);
				scribe->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
				scribe->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
				scribe->addChild(parent->getChild(i)->asTransform()->getChild(0));
				parent->getChild(i)->asTransform()->replaceChild(parent->getChild(i)->asTransform()->getChild(0), scribe);
			}
		}
		// already highlighted, take it away
		else {
			for (int i = 2; i < parent->getNumChildren(); i++) {
				osgFX::Scribe *parentScribe = dynamic_cast<osgFX::Scribe *>(parent->getChild(i)->asTransform()->getChild(0));
				parent->getChild(i)->asTransform()->replaceChild(parentScribe, parentScribe->getChild(0));
			}
		}
	}
	else if (parent->getName() == "ground") {
		// not highlighted yet, do that now
		if (!(dynamic_cast<osgFX::Scribe *>(child))) {
			osgFX::Scribe *scribe = new osgFX::Scribe();
			scribe->setWireframeColor(osg::Vec4(1, 1, 0, 0.1));
			scribe->setWireframeLineWidth(1);
			scribe->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			scribe->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			scribe->addChild(parent->getChild(0)->asTransform()->getChild(0));
			parent->getChild(0)->asTransform()->replaceChild(parent->getChild(0)->asTransform()->getChild(0), scribe);
		}
		// already highlighted, take it away
		else {
			osgFX::Scribe *parentScribe = dynamic_cast<osgFX::Scribe *>(parent->getChild(0)->asTransform()->getChild(0));
			parent->getChild(0)->asTransform()->replaceChild(parentScribe, parentScribe->getChild(0));
		}
	}
}

void Scene::toggleLabel(osg::Group *parent, osg::Node *child) {
	if (!_label) return;

	if (parent->getName() == "robot") {
		osg::Geode *geode = dynamic_cast<osg::Geode *>(parent->getChild(0));
		geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
	}
	else if (parent->getName() == "ground") {
	}
}

/**********************************************************
	private functions
 **********************************************************/
/*int Scene::draw(rsRobots::Cubus *robot, int tracke, double *rgb) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body[robot->NUM_PARTS];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[robot->NUM_PARTS];
	osg::ref_ptr<osg::Texture2D> tex[2];
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		body[i] = new osg::Geode;
	}

	// body
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_width, robot->_body_length, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY]->addDrawable(new osg::ShapeDrawable(box));

	// 'led'
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]+0.0001), 0.01, robot->_body_height);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	_robot.back()->led = new osg::ShapeDrawable(cyl);
	_robot.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY]->addDrawable(_robot.back()->led);

	// face1
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE1][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE1][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE1]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 2
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE2][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE2][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE2]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 3
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE3][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE3][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE3]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 4
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE4][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE4][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE4]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 5
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE5][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE5][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE5]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 6
	pos = dGeomGetOffsetPosition(robot->_geom[robot->FACE6][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->FACE6][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->FACE6]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex[0] = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/body.png"));
	tex[1] = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/face.png"));
	for (int i = 0; i < 2; i++) {
		tex[i]->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex[i]->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex[i]->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex[i]->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	}
	body[robot->BODY]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	body[robot->FACE1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[robot->FACE2]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[robot->FACE3]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[robot->FACE4]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[robot->FACE5]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[robot->FACE6]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);

	// set rendering properties
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// position each body within robot
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robot.back()->robot->addChild(pat[i].get());
	}

	// add connectors
	for (int i = 0; i < robot->_conn.size(); i++) {
		// initialize variables
		dMatrix3 R;
		dQuaternion Q;
		double p[3] = {0};

		// get connection parameters
		robot->getFaceParams(robot->_conn[i]->face, R, p);
		dRtoQ(R, Q);
		if (robot->_conn[i]->d_side != -1) robot->getConnectorParams(robot->_conn[i]->d_type, robot->_conn[i]->d_side, R, p);

		// PAT to transform mesh
		osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
		transform->setPosition(osg::Vec3d(p[0], p[1], p[2]));
		transform->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));

		// create node to hold mesh
		osg::ref_ptr<osg::Node> geode = osgDB::readNodeFile(_tex_path + "linkbot/models/simple.3ds");
		geode->setCullingActive(false);

		// apply texture
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/conn.png"));
		tex->setDataVariance(osg::Object::DYNAMIC);
		tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
		tex->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
		geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
		osg::ref_ptr<osg::TexEnv> texEnv = new osg::TexEnv(osg::TexEnv::DECAL);
		geode->getOrCreateStateSet()->setTextureAttribute(0, texEnv, osg::StateAttribute::ON);

		// set rendering
		geode->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		// add body to pat
		transform->addChild(geode);

		// set user properties of node
		geode->setName("connector");

		// add to scenegraph
		_robot.back()->robot->addChild(transform);
	}

	// add sensors
	//for (int i = 0; i < _sensor.size(); i++) {
		//this->drawSensor(i, _robot);
	//}

	// set update callback for robot
	_robot.back()->robot->setUpdateCallback(new cubusNodeCallback(robot, _robot.back()->led));

	// set tracking
	robot->_trace = trace;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}*/

void Scene::draw_linkbot(rsRobots::LinkbotT *robot, Robot *group, const double *p, const double *q, bool trace, double *rgb) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body[rsRobots::LinkbotT::NUM_PARTS];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[rsRobots::LinkbotT::NUM_PARTS];
	osg::Box *box;
	osg::Cylinder *cyl;
	double p1[3], q1[4];

	// build bodies
	for (int i = 0; i < rsRobots::LinkbotT::NUM_PARTS; i++) {
		body[i] = new osg::Geode;
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i]);
		group->addChild(pat[i].get());
	}

	// get led color
	rgb = robot->getRGB();

	// body
	box = new osg::Box(osg::Vec3d(0, 0, 0), 0.07835, 0.03935, 0.07250);
	cyl = new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.03625, 0.07835);
	body[rsRobots::LinkbotT::BODY]->addDrawable(new osg::ShapeDrawable(box));
	body[rsRobots::LinkbotT::BODY]->addDrawable(new osg::ShapeDrawable(cyl));
	pat[rsRobots::LinkbotT::BODY]->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat[rsRobots::LinkbotT::BODY]->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));

	// 'led'
	cyl = new osg::Cylinder(osg::Vec3d(0, 0, 0 + 0.0001), 0.01, 0.07250);
	osg::ShapeDrawable *led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(rgb[0], rgb[1], rgb[2], 1));
	body[rsRobots::LinkbotT::BODY]->addDrawable(led);

	// face1
	robot->getRobotBodyOffset(rsRobots::LinkbotT::FACE1, p, q, p1, q1);
	cyl = new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.03060, 0.00200);
	body[rsRobots::LinkbotT::FACE1]->addDrawable(new osg::ShapeDrawable(cyl));
	pat[rsRobots::LinkbotT::FACE1]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsRobots::LinkbotT::FACE1]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// face 2
	robot->getRobotBodyOffset(rsRobots::LinkbotT::FACE2, p, q, p1, q1);
	cyl = new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.03060, 0.00200);
	body[rsRobots::LinkbotT::FACE2]->addDrawable(new osg::ShapeDrawable(cyl));
	pat[rsRobots::LinkbotT::FACE2]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsRobots::LinkbotT::FACE2]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// face 3
	robot->getRobotBodyOffset(rsRobots::LinkbotT::FACE3, p, q, p1, q1);
	cyl = new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.03060, 0.00200);
	body[rsRobots::LinkbotT::FACE3]->addDrawable(new osg::ShapeDrawable(cyl));
	pat[rsRobots::LinkbotT::FACE3]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsRobots::LinkbotT::FACE3]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// apply texture to robot
	osg::ref_ptr<osg::Texture2D> tex[2];
	tex[0] = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/body.png"));
	tex[1] = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/face.png"));
	for (int i = 0; i < 2; i++) {
		tex[i]->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex[i]->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex[i]->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex[i]->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	}
	body[0]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	body[1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[2]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	body[3]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[1].get(), osg::StateAttribute::ON);
	switch (robot->getForm()) {
		case rs::LINKBOTI:
			body[rsRobots::LinkbotT::FACE2]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
			break;
		case rs::LINKBOTL:
			body[rsRobots::LinkbotT::FACE3]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
			break;
	}

	// rendering
	for (int i = 0; i < rsRobots::LinkbotT::NUM_PARTS; i++) {
		// set rendering properties
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// set tracking
	robot->setTrace(trace);
}

void Scene::draw_linkbot_conn(rsRobots::LinkbotT *robot, Robot *group, int type, int face, double size, int side, int conn) {
	// get robot p&q
	osg::PositionAttitudeTransform *pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsRobots::LinkbotT::BODY));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();
	double p1[3], p2[3], q1[4], q2[4];

	// get face p&q
	dynamic_cast<rsRobots::LinkbotI *>(robot)->getRobotFaceOffset(face, p.ptr(), q.asVec4().ptr(), p1, q1);
	if (side <= -10) {
		/* // initialize variables
		osg::ref_ptr<osg::Geode> body = new osg::Geode;
		osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
		dQuaternion quat;
		osg::Box *box;
		osg::Cylinder *cyl;
		osg::Sphere *sph;
		double	depth = robot->_conn_depth,
				width = 1.5*robot->_face_radius,
				height = robot->_body_height;

		pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
		dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
		box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), depth, width, height);
		box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		body->addDrawable(new osg::ShapeDrawable(box));
		pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[1]);
		dGeomGetOffsetQuaternion(robot->_conn[i]->geom[1], quat);
		box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.02, 0.022, 0.0032);
		box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		body->addDrawable(new osg::ShapeDrawable(box));
		pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[2]);
		dGeomGetOffsetQuaternion(robot->_conn[i]->geom[2], quat);
		cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), 0.011, robot->_radius - robot->_face_radius - 0.006 + 0.0032);
		cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		body->addDrawable(new osg::ShapeDrawable(cyl));
		pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[3]);
		dGeomGetOffsetQuaternion(robot->_conn[i]->geom[3], quat);
		sph = new osg::Sphere(osg::Vec3d(pos[0], pos[1], pos[2]), 0.006);
		body->addDrawable(new osg::ShapeDrawable(sph));

		// apply texture
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/conn.png"));
		tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
		pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

		// set rendering
		body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		// add body to pat
		pat->addChild(body.get());
		// set user properties of node
		body->setName("connector");
		// add to scenegraph
		_robot.back()->robot->addChild(pat);
		return 0;*/
	}
	else if (conn == -1)
		dynamic_cast<rsRobots::LinkbotI *>(robot)->getConnBodyOffset(type, p1, q1, p2, q2);
	else {
		dynamic_cast<rsRobots::LinkbotI *>(robot)->getConnFaceOffset(type, side, p1, q1, p2, q2);
		type = conn;
		dynamic_cast<rsRobots::LinkbotI *>(robot)->getConnBodyOffset(type, p2, q2, p2, q2);
	}

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(p2[0], p2[1], p2[2]));
	transform->setAttitude(osg::Quat(q2[0], q2[1], q2[2], q2[3]));

	// create node to hold mesh
	osg::ref_ptr<osg::Node> node;
	switch (type) {
		case rs::BIGWHEEL:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/bigwheel.3ds");
			break;
		case rs::BRIDGE:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/bridge.3ds");
			break;
		case rs::CASTER:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/caster.3ds");
			break;
		case rs::CUBE:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/cube.3ds");
			break;
		case rs::FACEPLATE:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/faceplate.3ds");
			break;
		case rs::GRIPPER:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/gripper.3ds");
			break;
		case rs::OMNIDRIVE:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/omnidrive.3ds");
			break;
		case rs::SIMPLE:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/simple.3ds");
			break;
		case rs::SMALLWHEEL:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/smallwheel.3ds");
			break;
		case rs::TINYWHEEL:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/tinywheel.3ds");
			break;
		case rs::WHEEL:
			node = osgDB::readNodeFile(_tex_path + "linkbot/models/tinywheel.3ds");
			//transform->setScale(osg::Vec3d(1, robot->_wheel_radius/robot->_tinywheel_radius, robot->_wheel_radius/robot->_tinywheel_radius));
			break;
	}
	node->setCullingActive(false);

	// apply texture
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/conn.png"));
	tex->setDataVariance(osg::Object::DYNAMIC);
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
	node->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexEnv> texEnv = new osg::TexEnv(osg::TexEnv::DECAL);
	node->getOrCreateStateSet()->setTextureAttribute(0, texEnv, osg::StateAttribute::ON);

	// set rendering
	node->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	node->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// add body to pat
	transform->addChild(node);

	// set user properties of node
	node->setName("connector");

	// add to scenegraph
	group->addChild(transform);
}

/*int Scene::draw(rsRobots::CMobot *robot, bool trace, double *rgb) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body[5];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[5];
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < 5; i++) {
		body[i] = new osg::Geode;
	}

	// left endcap
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_width - 2*robot->_end_radius, robot->_end_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][1]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_radius, robot->_end_height - 2*robot->_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][2]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_radius, robot->_end_height - 2*robot->_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][3]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][4]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][5]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_L][6]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_L][6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_L]->addDrawable(new osg::ShapeDrawable(cyl));

	// left body
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_L][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_L][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_end_depth, robot->_body_width, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_L][1]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_L][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_length, robot->_body_inner_width_left, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_L][2]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_L][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_length, robot->_body_inner_width_right, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_L]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_L][3]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_L][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_radius, robot->_body_inner_width_left);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_L]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_L][4]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_L][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_radius, robot->_body_inner_width_right);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_L]->addDrawable(new osg::ShapeDrawable(cyl));

	// center
	pos = dGeomGetOffsetPosition(robot->_geom[robot->CENTER][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->CENTER][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_center_length, robot->_center_width, robot->_center_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->CENTER]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->CENTER][1]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->CENTER][1], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_center_radius, robot->_center_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->CENTER]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->CENTER][2]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->CENTER][2], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_center_radius, robot->_center_width);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->CENTER]->addDrawable(new osg::ShapeDrawable(cyl));

    // right body
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_R][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_R][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_end_depth, robot->_body_width, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_R][1]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_R][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_length, robot->_body_inner_width_left, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_R][2]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_R][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_length, robot->_body_inner_width_right, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_R][3]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_R][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_radius, robot->_body_inner_width_left);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY_R][4]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY_R][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_radius, robot->_body_inner_width_right);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY_R]->addDrawable(new osg::ShapeDrawable(cyl));

	// 'led'
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]+0.0001), 0.01, robot->_body_height);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	_robot.back()->led = new osg::ShapeDrawable(cyl);
	_robot.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY_R]->addDrawable(_robot.back()->led);

	// right endcap
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_width - 2*robot->_end_radius, robot->_end_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][1]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][1], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_radius, robot->_end_height - 2*robot->_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][2]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][2], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_depth, robot->_end_radius, robot->_end_height - 2*robot->_end_radius);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(box));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][3]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][3], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][4]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][4], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][5]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][5], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));
	pos = dGeomGetOffsetPosition(robot->_geom[robot->ENDCAP_R][6]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->ENDCAP_R][6], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_radius, robot->_end_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->ENDCAP_R]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "mobot/body.png"));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
	body[robot->ENDCAP_L]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	body[robot->BODY_L]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	body[robot->CENTER]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	body[robot->BODY_R]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	body[robot->ENDCAP_R]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

	// set rendering properties
	for (int i = 0; i < 5; i++) {
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// position each body within robot
	for (int i = 0; i < 5; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robot.back()->robot->addChild(pat[i].get());
	}

	// add connectors
	for (int i = 0; i < robot->_conn.size(); i++) {
		osg::ref_ptr<osg::Geode> body = new osg::Geode;
		osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
		const double *pos;
		dQuaternion quat;
		osg::Cylinder *cyl;
		osg::Box *box;
		osg::Sphere *sph;

		switch (robot->_conn[i]->type) {
			case BIGWHEEL:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_bigwheel_radius, 2*robot->_conn_depth/3);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				break;
			case CASTER:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_end_width - 2*robot->_conn_radius, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[1]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[1], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_conn_radius, robot->_conn_height - 2*robot->_conn_radius);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[2]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[2], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_conn_radius, robot->_conn_height - 2*robot->_conn_radius);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[3]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[3], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[4]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[4], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[5]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[5], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[6]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[6], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[7]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[7], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0667, 0.0222, 0.0032);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[8]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[8], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0111, 0.0191);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[9]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[9], quat);
				sph = new osg::Sphere(osg::Vec3d(pos[0], pos[1], pos[2]), 0.0095);
				body->addDrawable(new osg::ShapeDrawable(sph));
				break;
			case SIMPLE:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_end_width - 2*robot->_conn_radius, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[1]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[1], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_conn_radius, robot->_conn_height - 2*robot->_conn_radius);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[2]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[2], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_conn_radius, robot->_conn_height - 2*robot->_conn_radius);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[3]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[3], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[4]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[4], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[5]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[5], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[6]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[6], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_radius, robot->_conn_depth);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				break;
			case SMALLWHEEL:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_smallwheel_radius, 2*robot->_conn_depth/3);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				break;
			case SQUARE:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_end_width, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[1]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[1], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_width - 2*robot->_conn_depth, robot->_conn_depth, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[2]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[2], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_end_width - 2*robot->_conn_depth, robot->_conn_depth, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[3]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[3], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_conn_depth, robot->_end_width, robot->_conn_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				break;
			case TANK:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_tank_depth, robot->_end_width, robot->_tank_height);
				box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(box));
				break;
			case WHEEL:
				pos = dGeomGetOffsetPosition(robot->_conn[i]->geom[0]);
				dGeomGetOffsetQuaternion(robot->_conn[i]->geom[0], quat);
				cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_wheel_radius, 2*robot->_conn_depth/3);
				cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
				body->addDrawable(new osg::ShapeDrawable(cyl));
				break;
		}
		// apply texture
		osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "mobot/conn.png"));
		tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
		tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
		tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
		pat->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);

		// set rendering
		body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin");
		body->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		// add body to pat
		pat->addChild(body.get());
		// optimize
		osgUtil::Optimizer optimizer;
		optimizer.optimize(pat);
		// add to scenegraph
		_robot.back()->robot->addChild(pat);
	}

	// set update callback for robot
	_robot.back()->robot->setUpdateCallback(new mobotNodeCallback(robot, _robot.back()->led));

	// set tracking
	robot->_trace = trace;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}

int Scene::draw(rsRobots::CNXT *robot, bool trace, double *rgb) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body[robot->NUM_PARTS];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[robot->NUM_PARTS];
	osg::ref_ptr<osg::Texture2D> tex;
	const double *pos;
	dQuaternion quat;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		body[i] = new osg::Geode;
	}

	// body
	pos = dGeomGetOffsetPosition(robot->_geom[robot->BODY][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->BODY][0], quat);
	box = new osg::Box(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_body_width, robot->_body_length, robot->_body_height);
	box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->BODY]->addDrawable(new osg::ShapeDrawable(box));

	// 'led'
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]+0.0001), 0.01, robot->_body_height);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	_robot.back()->led = new osg::ShapeDrawable(cyl);
	_robot.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY]->addDrawable(_robot.back()->led);

	// wheel1
	pos = dGeomGetOffsetPosition(robot->_geom[robot->WHEEL1][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->WHEEL1][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_wheel_radius, robot->_wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->WHEEL1]->addDrawable(new osg::ShapeDrawable(cyl));

	// wheel2
	pos = dGeomGetOffsetPosition(robot->_geom[robot->WHEEL2][0]);
	dGeomGetOffsetQuaternion(robot->_geom[robot->WHEEL2][0], quat);
	cyl = new osg::Cylinder(osg::Vec3d(pos[0], pos[1], pos[2]), robot->_wheel_radius, robot->_wheel_depth);
	cyl->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	body[robot->WHEEL2]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
	tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "linkbot/textures/body.png"));
	tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

	for (int i = 0; i < robot->NUM_PARTS; i++) {
		// set rendering properties
		body[i]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		// position in robot
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robot.back()->robot->addChild(pat[i].get());
	}

	// set update callback for robot
	_robot.back()->robot->setUpdateCallback(new nxtNodeCallback(robot, _robot.back()->led));

	// set tracking
	robot->_trace = trace;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}*/

void* Scene::graphics_thread(void *arg) {
	// cast viewer
	Scene *p = (Scene *)arg;

	// viewer event handlers
	p->_viewer->addEventHandler(new osgViewer::WindowSizeHandler);

	// run viewer
	MUTEX_LOCK(&(p->_thread_mutex));
	while (p->_thread && !p->_viewer->done()) {
		MUTEX_UNLOCK(&(p->_thread_mutex));

		p->_viewer->frame();
		p->addChild();
		if (p->_deleting) {
			p->_scene->removeChild(p->_scene->getChild(p->_deleting));
			p->_deleting = 0;
		}

		MUTEX_LOCK(&(p->_thread_mutex));
	}
	MUTEX_UNLOCK(&(p->_thread_mutex));

	// clean up viewer & root
	p->_viewer->setSceneData(NULL);
#ifdef _WIN32_
	delete p->_viewer;
#endif

	// trigger end of code when graphics window is closed
	//g_sim->done();

	// return
	return arg;
}

