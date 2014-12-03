#include "graphics.hpp"
#include "robosim.hpp"
#include "linkbot.hpp"
#include "mobot.hpp"
#include "nxt.hpp"
#include "cubus.hpp"

osg::Node::NodeMask NOT_VISIBLE_MASK = 0x0;
osg::Node::NodeMask RECEIVES_SHADOW_MASK = 0x1;
osg::Node::NodeMask CASTS_SHADOW_MASK = 0x2;
osg::Node::NodeMask IS_PICKABLE_MASK = 0x3;
osg::Node::NodeMask VISIBLE_MASK = 0xffffffff;

/**********************************************************
	Graphics
 **********************************************************/
Graphics::Graphics(RoboSim *sim) {
	// set notification level to no output
	osg::setNotifyLevel(osg::ALWAYS);

    // build the viewer
	MUTEX_INIT(&_viewer_mutex);
	_viewer = 1;

	// separate viewer and inserting into scenegraph
	_staging = new osg::Group;
	_ending = 0;

	// set default grid options
	_us = 1;			// customary units
	_grid[0] = 1;		// 1 inch per tic
	_grid[1] = 12;		// 12 inches per hash
	_grid[2] = -24;		// min x
	_grid[3] = 24;		// max x
	_grid[4] = -24;		// min y
	_grid[5] = 24;		// max y
	_grid[6] = 1;		// enabled or not
	for (int i = 0; i < 6; i++) {
		if (_us) _grid[i] /= 39.37;
		else _grid[i] /= 100;
	}

	// set texture path
	_tex_path = this->getTexPath();

	_sim = sim;
}

Graphics::~Graphics(void) {
	MUTEX_LOCK(&_viewer_mutex);
	_viewer = 0;
	MUTEX_UNLOCK(&_viewer_mutex);
	THREAD_JOIN(_osgThread);
	COND_DESTROY(&_graphics_cond);
	MUTEX_DESTROY(&_graphics_mutex);
	MUTEX_DESTROY(&_viewer_mutex);

	// remove drawings
	for (int i = 0; i < _drawings.size(); i++) {
		delete _drawings[i];
	}
}

int Graphics::addChild(void) {
	if (_staging->getNumChildren()) {
		_scene->addChild(_staging->getChild(0));
		_staging->removeChild(0, 1);
	}

	// success
	return 0;
}

int Graphics::drawGround(Ground *object) {
	osg::Box *box;
	osg::Cylinder *cyl;
	osg::Sphere *sph;

	osg::ref_ptr<osg::Group> ground = new osg::Group();
	osg::ref_ptr<osg::Geode> body = new osg::Geode;
	osg::ref_ptr<osg::ShapeDrawable> shape;
	switch (object->type) {
		case BOX:
			box = new osg::Box(osg::Vec3d(object->x, object->y, object->z), object->l1, object->l2, object->l3);
			box->setRotation(osg::Quat(object->q[1], object->q[2], object->q[3], object->q[0]));
			shape = new osg::ShapeDrawable(box);
			break;
		case CYLINDER:
			cyl = new osg::Cylinder(osg::Vec3d(object->x, object->y, object->z), object->l1, object->l2);
			cyl->setRotation(osg::Quat(object->q[1], object->q[2], object->q[3], object->q[0]));
			shape = new osg::ShapeDrawable(cyl);
			break;
		case SPHERE:
			sph = new osg::Sphere(osg::Vec3d(object->x, object->y, object->z), object->l1);
			shape = new osg::ShapeDrawable(sph);
			break;
	}
	shape->setColor(osg::Vec4(object->r, object->g, object->b, object->alpha));
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
	pat->addChild(body.get());
	ground->addChild(pat.get());

	// add update callback
	//ground->setUpdateCallback(new groundNodeCallback(object));

	// set user properties of node
	ground->setName("ground");

	// optimize object
	osgUtil::Optimizer optimizer;
	optimizer.optimize(ground);

	// add to scenegraph
	_staging->addChild(ground);

	// success
	return 0;
}

int Graphics::drawMarker(Drawing *marker) {
	// create geode
	osg::Geode *geode = new osg::Geode();

	// draw specific marker
	if (marker->type == DOT) {
		osg::Sphere *sphere = new osg::Sphere(osg::Vec3d(marker->p1[0], marker->p1[1], marker->p1[2]), marker->i/500.0);
		osg::ShapeDrawable *pointDrawable = new osg::ShapeDrawable(sphere);
		pointDrawable->setColor(osg::Vec4(marker->c[0], marker->c[1], marker->c[2], marker->c[3]));
		geode->addDrawable(pointDrawable);
	}
	else if (marker->type == LINE) {
		osg::Geometry *geom = new osg::Geometry();
		osg::Vec3Array *vert = new osg::Vec3Array();
		vert->push_back(osg::Vec3(marker->p1[0], marker->p1[1], marker->p1[2]));
		vert->push_back(osg::Vec3(marker->p2[0], marker->p2[1], marker->p2[2]));
		geom->setVertexArray(vert);
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
		osg::Vec4Array *colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(marker->c[0], marker->c[1], marker->c[2], marker->c[3]));
		geom->setColorArray(colors);
		geom->setColorBinding(osg::Geometry::BIND_OVERALL);
		osg::LineWidth *width = new osg::LineWidth();
		width->setWidth(marker->i*3.0f);
		geode->addDrawable(geom);
		geode->getOrCreateStateSet()->setAttributeAndModes(width, osg::StateAttribute::ON);
	}
	else if (marker->type == TEXT) {
		osgText::Text *label = new osgText::Text();
		label->setAlignment(osgText::Text::CENTER_CENTER);
		label->setAxisAlignment(osgText::Text::SCREEN);
		label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		label->setCharacterSize(25);
		label->setColor(osg::Vec4(marker->c[0], marker->c[1], marker->c[2], marker->c[3]));
		label->setDrawMode(osgText::Text::TEXT);
		label->setPosition(osg::Vec3(marker->p1[0], marker->p1[1], marker->p1[2]));
		label->setText(marker->str);
		geode->addDrawable(label);
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

int Graphics::drawRobot(Robot *robot, XMLRobot *xml, int type, int tracking) {
	_robots.push_back(new GRobot);
	_robots.back()->robot = new osg::Group();
	double rgb[4] = {0};

	switch (type) {
		case CUBUS:
			this->draw(dynamic_cast<Cubus*>(robot), tracking, rgb);
			break;
		case LINKBOTI:
			this->draw(dynamic_cast<CLinkbotI*>(robot), xml, tracking, rgb);
			break;
		case LINKBOTL:
			this->draw(dynamic_cast<CLinkbotL*>(robot), xml, tracking, rgb);
			break;
		case LINKBOTT:
			this->draw(dynamic_cast<CLinkbotT*>(robot), xml, tracking, rgb);
			break;
		case MOBOT:
			this->draw(dynamic_cast<CMobot*>(robot), tracking, rgb);
			break;
		case NXT:
			this->draw(dynamic_cast<CNXT*>(robot), tracking, rgb);
			break;
	}

	// set masks
	//robot->setNodeMask(CASTS_SHADOW_MASK);
	//robot->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	osgText::Text *label = new osgText::Text();
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
	label->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
	label->setBoundingBoxColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.9f));
	label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	label->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX);
	_robots.back()->robot->insertChild(0, label_geode);

	// draw tracking node
	osg::Geode *trackingGeode = new osg::Geode();
	osg::Geometry *trackingLine = new osg::Geometry();
	osg::Vec3Array *trackingVertices = new osg::Vec3Array();
	trackingGeode->setNodeMask((tracking) ? VISIBLE_MASK : NOT_VISIBLE_MASK);
	trackingLine->setVertexArray(trackingVertices);
	trackingLine->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1, 1));
	trackingLine->setDataVariance(osg::Object::DYNAMIC);
	trackingLine->setUseDisplayList(false);
	osg::Vec4Array *colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(rgb[0], rgb[1], rgb[2], rgb[3]));
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
	//_robots.back()->robot->insertChild(1, trackingGeode);

	// set user properties of node
	_robots.back()->robot->setName("robot");

	// optimize robot
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_robots.back()->robot);

	// add to scenegraph
	_staging->addChild(_robots.back()->robot);

	// return position of robot in root node
	return (_staging->getChildIndex(_robots.back()->robot));
}

osgText::Text* Graphics::getHUDText(void) {
	osg::Geode *geode = _scene->getParent(0)->getChild(1)->asGroup()->getChild(0)->asTransform()->getChild(0)->asGeode();
	return dynamic_cast<osgText::Text *>(geode->getDrawable(0));
}

std::string Graphics::getTexPath(void) {
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
	_tex_path = path;
	return path;
}

int Graphics::getUnits(void) {
	return _us;
}

/*void Graphics::readXML(tinyxml2::XMLDocument *doc) {
	tinyxml2::XMLElement *node = NULL;
	tinyxml2::XMLElement *ele = NULL;
	tinyxml2::XMLElement *side = NULL;
	int tracking = 0;

	// check for existence of graphics node
	if ( (node = doc->FirstChildElement("graphics")) ) {
		node = node->FirstChildElement();
	}

	// loop over all graphics nodes ignoring grid
	while (node) {
		if ( !strcmp(node->Value(), "line") ) {
			// store default variables
			_drawings.push_back(new Drawing());
			_drawings.back()->type = LINE;

			// get user defined values from xml
			if (node->QueryIntAttribute("width", &_drawings.back()->i)) {
				_drawings.back()->i = 1;
			}
			if ( (ele = node->FirstChildElement("start")) ) {
				ele->QueryDoubleAttribute("x", &_drawings.back()->p1[0]);
				ele->QueryDoubleAttribute("y", &_drawings.back()->p1[1]);
				ele->QueryDoubleAttribute("z", &_drawings.back()->p1[2]);
			}
			if ( (ele = node->FirstChildElement("end")) ) {
				ele->QueryDoubleAttribute("x", &_drawings.back()->p2[0]);
				ele->QueryDoubleAttribute("y", &_drawings.back()->p2[1]);
				ele->QueryDoubleAttribute("z", &_drawings.back()->p2[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_drawings.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_drawings.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_drawings.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_drawings.back()->c[3]);
			}
		}
		else if ( !strcmp(node->Value(), "point") ) {
			// store default variables
			_drawings.push_back(new Drawing());
			_drawings.back()->type = DOT;

			// get user defined values from xml
			if (node->QueryIntAttribute("size", &_drawings.back()->i)) {
				_drawings.back()->i = 1;
			}
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &_drawings.back()->p1[0]);
				ele->QueryDoubleAttribute("y", &_drawings.back()->p1[1]);
				ele->QueryDoubleAttribute("z", &_drawings.back()->p1[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_drawings.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_drawings.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_drawings.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_drawings.back()->c[3]);
			}
		}
		else if ( !strcmp(node->Value(), "grid") ) {
			node->QueryIntAttribute("units", &_us);
			node->QueryDoubleAttribute("tics", &_grid[0]);
			node->QueryDoubleAttribute("major", &_grid[1]);
			node->QueryDoubleAttribute("minx", &_grid[2]);
			node->QueryDoubleAttribute("maxx", &_grid[3]);
			node->QueryDoubleAttribute("miny", &_grid[4]);
			node->QueryDoubleAttribute("maxy", &_grid[5]);
			node->QueryDoubleAttribute("enabled", &_grid[6]);
			for (int i = 0; i < 6; i++) {
				if (_us)_grid[i] /= 39.37;
				else _grid[i] /= 100;
			}
		}
		else if ( !strcmp(node->Value(), "tracking") ) {
			node->QueryIntAttribute("val", &tracking);
		}
		else {
			// store default variables
			_drawings.push_back(new Drawing());
			_drawings.back()->type = TEXT;

			// get user defined values from xml
			_drawings.back()->str = node->Value();
			if ( (ele = node->FirstChildElement("position")) ) {
				ele->QueryDoubleAttribute("x", &_drawings.back()->p1[0]);
				ele->QueryDoubleAttribute("y", &_drawings.back()->p1[1]);
				ele->QueryDoubleAttribute("z", &_drawings.back()->p1[2]);
			}
			if ( (ele = node->FirstChildElement("color")) ) {
				ele->QueryDoubleAttribute("r", &_drawings.back()->c[0]);
				ele->QueryDoubleAttribute("g", &_drawings.back()->c[1]);
				ele->QueryDoubleAttribute("b", &_drawings.back()->c[2]);
				ele->QueryDoubleAttribute("alpha", &_drawings.back()->c[3]);
			}
		}
		// go to next node
		node = node->NextSiblingElement();
	}
}*/

int Graphics::stageForDelete(int id) {
	_ending = id;

	// success
	return 0;
}

void Graphics::start(int pause) {
	// graphics haven't started yet
	COND_INIT(&_graphics_cond);
	MUTEX_INIT(&_graphics_mutex);
	_graphics = 0;
	_ending = pause;

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
	this->setupCamera(gc.get(), _view, traits->width, traits->height);

	// set up scene
	this->setupScene(_view, traits->width, traits->height);

	// create graphics thread
	THREAD_CREATE(&_osgThread, (void* (*)(void *))&Graphics::graphics_thread, (void *)this);

	// wait for graphics to be ready
	MUTEX_LOCK(&_graphics_mutex);
	while (!_graphics) {
		COND_WAIT(&_graphics_cond, &_graphics_mutex);
	}
	MUTEX_UNLOCK(&_graphics_mutex);
	_ending = 0;
}

int Graphics::setupCamera(osg::GraphicsContext *gc, osgViewer::Viewer *viewer, double w, double h) {
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
	viewer->addSlave(_camera);

	// viewer camera properties
	osg::ref_ptr<osgGA::OrbitManipulator> cameraManipulator = new osgGA::OrbitManipulator();
	cameraManipulator->setDistance(0.1);
	cameraManipulator->setAllowThrow(false);
	cameraManipulator->setWheelZoomFactor(0);
	cameraManipulator->setVerticalAxisFixed(true);
	cameraManipulator->setElevation(0.5);
	viewer->setCameraManipulator(cameraManipulator);
	viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.6, -0.8, 0.5), osg::Vec3f(0.1, 0.3, 0), osg::Vec3f(0, 0, 1));

	// event handler
	viewer->addEventHandler(new osgGA::StateSetManipulator(_camera->getOrCreateStateSet()));

	// success
	return 0;
}

int Graphics::setupScene(osgViewer::Viewer *viewer, double w, double h) {
	Graphics *sim = this;

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
	osg::ref_ptr<osg::Node> t_geode = osgDB::readNodeFile(sim->_tex_path + "ground/terrain.3ds");
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
	if (sim->_ending) textHUD->setText("Paused: Press any key to start");
	textHUD->setAxisAlignment(osgText::Text::SCREEN);
	textHUD->setAlignment(osgText::Text::CENTER_CENTER);
	textHUD->setPosition(osg::Vec3(w/2, 50, -1.5));
	textHUD->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	textHUD->setBackdropColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	textHUD->setBackdropOffset(0.1);
	_root->addChild(HUDProjectionMatrix);

	// grid
	if ( (int)(sim->_grid[6]) ) {
		// grid lines for each sub-foot
		double minx = (int)((sim->_grid[2]*1.01)/sim->_grid[0])*sim->_grid[0];
		double miny = (int)((sim->_grid[4]*1.01)/sim->_grid[0])*sim->_grid[0];
		int numx = (int)((sim->_grid[3] - minx)/sim->_grid[0]+1);
		int numy = (int)((sim->_grid[5] - miny)/sim->_grid[0]+1);
		int numVertices = 2*numx + 2*numy;
		osg::Geode *gridGeode = new osg::Geode();
		osg::Geometry *gridLines = new osg::Geometry();
		osg::Vec3 *myCoords = new osg::Vec3[numVertices]();
		// draw x lines
		for (int i = 0, j = 0; i < numx; i++) {
			myCoords[j++] = osg::Vec3(minx + i*sim->_grid[0], sim->_grid[4], 0.0);
			myCoords[j++] = osg::Vec3(minx + i*sim->_grid[0], sim->_grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx; i < numy; i++) {
			myCoords[j++] = osg::Vec3(sim->_grid[2], miny + i*sim->_grid[0], 0.0);
			myCoords[j++] = osg::Vec3(sim->_grid[3], miny + i*sim->_grid[0], 0.0);
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
		double minx2 = (int)((sim->_grid[2]*1.01)/sim->_grid[1])*sim->_grid[1];
		double miny2 = (int)((sim->_grid[4]*1.01)/sim->_grid[1])*sim->_grid[1];
		int numx2 = (int)((sim->_grid[3] - minx2)/sim->_grid[1]+1);
		int numy2 = (int)((sim->_grid[5] - miny2)/sim->_grid[1]+1);
		int numVertices2 = 2*numx2 + 2*numy2;
		osg::Geode *gridGeode2 = new osg::Geode();
		osg::Geometry *gridLines2 = new osg::Geometry();
		osg::Vec3 *myCoords2 = new osg::Vec3[numVertices2]();
		// draw x lines
		for (int i = 0, j = 0; i < numx2; i++) {
			myCoords2[j++] = osg::Vec3(minx2 + i*sim->_grid[1], sim->_grid[4], 0.0);
			myCoords2[j++] = osg::Vec3(minx2 + i*sim->_grid[1], sim->_grid[5], 0.0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx2; i < numy2; i++) {
			myCoords2[j++] = osg::Vec3(sim->_grid[2], miny2 + i*sim->_grid[1], 0.0);
			myCoords2[j++] = osg::Vec3(sim->_grid[3], miny2 + i*sim->_grid[1], 0.0);
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
		if ( fabs(sim->_grid[3]) > fabs(sim->_grid[2]) ) {
			if (sim->_grid[2] < -EPSILON)
				myCoords3[0] = osg::Vec3(-sim->_grid[3], 0, 0);
			else
				myCoords3[0] = osg::Vec3(0, 0, 0);
			myCoords3[1] = osg::Vec3(sim->_grid[3], 0, 0);
		}
		else {
			if (sim->_grid[3] < -EPSILON)
				myCoords3[1] = osg::Vec3(0, 0, 0);
			else
				myCoords3[1] = osg::Vec3(sim->_grid[3], 0, 0);
			myCoords3[0] = osg::Vec3(sim->_grid[2], 0, 0);
		}
		if ( fabs(sim->_grid[5]) > fabs(sim->_grid[4]) ) {
			if (sim->_grid[4] < -EPSILON)
				myCoords3[2] = osg::Vec3(-sim->_grid[5], 0, 0);
			else
				myCoords3[2] = osg::Vec3(0, 0, 0);
			myCoords3[3] = osg::Vec3(0, sim->_grid[5], 0);
		}
		else {
			if (sim->_grid[5] < -EPSILON)
				myCoords3[3] = osg::Vec3(0, 0, 0);
			else
				myCoords3[3] = osg::Vec3(0, sim->_grid[5], 0);
			myCoords3[2] = osg::Vec3(0, sim->_grid[4], 0);
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
		if ( fabs(sim->_grid[3]) > fabs(sim->_grid[2]) ) {
			if (sim->_grid[3] < EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(0.05, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext, osg::Vec3d(sim->_grid[3] + 0.05, 0.0, 0.0));
		}
		else {
			if (sim->_grid[2] < -EPSILON)
				xbillboard->addDrawable(xtext, osg::Vec3d(sim->_grid[3] + 0.05, 0.0, 0.0));
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
		if ( fabs(sim->_grid[5]) > fabs(sim->_grid[4]) ) {
			if (sim->_grid[5] < EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, 0.05, 0.0));
			else
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, sim->_grid[5] + 0.05, 0.0));
		}
		else {
			if (sim->_grid[4] < -EPSILON)
				xbillboard->addDrawable(ytext, osg::Vec3d(0.0, sim->_grid[5] + 0.05, 0.0));
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
		xnum_billboard->addDrawable(xzero_text, osg::Vec3d(-0.5*sim->_grid[0], -0.5*sim->_grid[0], 0.0));
		// positive
		for (int i = 1; i < (int)(sim->_grid[3]/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumpos_text = new osgText::Text();
			if (sim->_us) sprintf(text, "   %.0lf", 39.37*i*sim->_grid[1]);
			else sprintf(text, "   %.0lf", 100*i*sim->_grid[1]);
			xnumpos_text->setText(text);
			xnumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumpos_text->setCharacterSize(30);
			xnumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumpos_text, osg::Vec3d(i*sim->_grid[1], -0.5*sim->_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(sim->_grid[2])/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> xnumneg_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", -39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", -100*i*sim->_grid[1]);
			xnumneg_text->setText(text);
			xnumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			xnumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			xnumneg_text->setCharacterSize(30);
			xnumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			xnumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			xnum_billboard->addDrawable(xnumneg_text, osg::Vec3d(-i*sim->_grid[1], -0.5*sim->_grid[0], 0.0));
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
		for (int i = 1; i < (int)(sim->_grid[5]/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumpos_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", 39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", 100*i*sim->_grid[1]);
			ynumpos_text->setText(text);
			ynumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumpos_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumpos_text->setCharacterSize(30);
			ynumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumpos_text, osg::Vec3d(0, i*sim->_grid[1] + 0.5*sim->_grid[0], 0.0));
		}
		// negative
		for (int i = 1; i < (int)(fabs(sim->_grid[4])/sim->_grid[1]+1); i++) {
			osg::ref_ptr<osgText::Text> ynumneg_text = new osgText::Text();
			if (sim->_us) sprintf(text, "%.0lf   ", -39.37*i*sim->_grid[1]);
			else sprintf(text, "%.0lf   ", -100*i*sim->_grid[1]);
			ynumneg_text->setText(text);
			ynumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			ynumneg_text->setAlignment(osgText::Text::CENTER_CENTER);
			ynumneg_text->setCharacterSize(30);
			ynumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
			ynumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
			ynum_billboard->addDrawable(ynumneg_text, osg::Vec3d(0, -i*sim->_grid[1] - 0.5*sim->_grid[0], 0.0));
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
	osg::Image* imagePosX = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_right.png");
	osg::Image* imageNegX = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_left.png");
	osg::Image* imagePosY = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_top.png");
	osg::Image* imageNegY = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_top.png");
	osg::Image* imagePosZ = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_front.png");
	osg::Image* imageNegZ = osgDB::readImageFile(sim->_tex_path + "ground/checkered/checkered_back.png");
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
	osg::ref_ptr<osg::Transform> transform = new MoveEarthySkyWithEyePointTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);
	osg::ref_ptr<osg::ClearNode> clearNode = new osg::ClearNode;
	clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TexMatCallback(*tm));
	clearNode->addChild(transform);
	clearNode->setNodeMask(~IS_PICKABLE_MASK);
	_root->addChild(clearNode);

	// drawing objects
	for (int i = 0; i < sim->_drawings.size(); i++) {
		sim->drawMarker(sim->_drawings[i]);
	}

	// optimize the scene graph, remove redundant nodes and state etc.
	osgUtil::Optimizer optimizer;
	optimizer.optimize(_root);

	// event handler
	viewer->addEventHandler(new keyboardEventHandler(textHUD));
	viewer->addEventHandler(new pickHandler());

	// show scene
	viewer->setSceneData(_root);

	// success
	return 0;
}

int Graphics::setupViewer(osgViewer::Viewer *viewer) {
    // creating the viewer
	if (viewer)
		_view = viewer;
	else
		_view = new osgViewer::Viewer;

	// set threading model
	_view->setThreadingModel(osgViewer::Viewer::SingleThreaded);

	// event handler
	_view->addEventHandler(new osgViewer::StatsHandler);


//tinyxml2::XMLDocument doc;
//doc.LoadFile("/home/kgucwa/projects/playground/RS/RoboSim/robosimrc");
//this->readXML(&doc);

	// success
	return 0;
}

/**********************************************************
	private functions
 **********************************************************/
int Graphics::draw(Cubus *robot, int tracking, double *rgb) {
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
	_robots.back()->led = new osg::ShapeDrawable(cyl);
	_robots.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY]->addDrawable(_robots.back()->led);

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
		_robots.back()->robot->addChild(pat[i].get());
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
		_robots.back()->robot->addChild(transform);
	}

	// add sensors
	//for (int i = 0; i < _sensor.size(); i++) {
		//this->drawSensor(i, _robot);
	//}

	// set update callback for robot
	_robots.back()->robot->setUpdateCallback(new cubusNodeCallback(robot, _robots.back()->led));

	// set tracking
	robot->_trace = tracking;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}

int Graphics::draw(CLinkbotT *robot, XMLRobot *xml, int tracking, double *rgb) {
	// initialize variables
	osg::ref_ptr<osg::Geode> body[4];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[4];
	osg::ref_ptr<osg::Texture2D> tex[2];
	const double *p;
	dQuaternion q;
	osg::Box *box;
	osg::Cylinder *cyl;
	for (int i = 0; i < 4; i++) {
		body[i] = new osg::Geode;
	}

	// body
	p = dGeomGetPosition(robot->_geom[0][0]);
	dGeomGetQuaternion(robot->_geom[0][0], q);
	box = new osg::Box(osg::Vec3d(p[0], p[1], p[2]), robot->_body_width, robot->_body_length, robot->_body_height);
	box->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(box));
	p = dGeomGetPosition(robot->_geom[0][1]);
	dGeomGetQuaternion(robot->_geom[0][1], q);
	cyl = new osg::Cylinder(osg::Vec3d(p[0], p[1], p[2]), robot->_body_radius, robot->_body_width);
	cyl->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	body[0]->addDrawable(new osg::ShapeDrawable(cyl));

	// 'led'
	cyl = new osg::Cylinder(osg::Vec3d(p[0], p[1], p[2] + 0.0001), 0.01, robot->_body_height);
	cyl->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	_robots.back()->led = new osg::ShapeDrawable(cyl);
	_robots.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[0]->addDrawable(_robots.back()->led);

	// face1
	p = dGeomGetPosition(robot->_geom[1][0]);
	dGeomGetQuaternion(robot->_geom[1][0], q);
	cyl = new osg::Cylinder(osg::Vec3d(p[0], p[1], p[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	body[1]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 2
	p = dGeomGetPosition(robot->_geom[2][0]);
	dGeomGetQuaternion(robot->_geom[2][0], q);
	cyl = new osg::Cylinder(osg::Vec3d(p[0], p[1], p[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	body[2]->addDrawable(new osg::ShapeDrawable(cyl));

	// face 3
	p = dGeomGetPosition(robot->_geom[3][0]);
	dGeomGetQuaternion(robot->_geom[3][0], q);
	cyl = new osg::Cylinder(osg::Vec3d(p[0], p[1], p[2]), robot->_face_radius, robot->_face_depth);
	cyl->setRotation(osg::Quat(q[1], q[2], q[3], q[0]));
	body[3]->addDrawable(new osg::ShapeDrawable(cyl));

	// apply texture to robot
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
	if (robot->_disabled > 0) {
		body[robot->_disabled+1]->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex[0].get(), osg::StateAttribute::ON);
	}

	// set rendering properties
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		body[i]->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
		body[i]->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	}

	// position each body within robot
	for (int i = 0; i < robot->NUM_PARTS; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		pat[i]->addChild(body[i].get());
		_robots.back()->robot->addChild(pat[i].get());
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
		if (robot->_conn[i]->d_side <= -10) {
			// initialize variables
			osg::ref_ptr<osg::Geode> body = new osg::Geode;
			osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
			const double *pos;
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
			_robots.back()->robot->addChild(pat);
			return 0;
		}
		else if (robot->_conn[i]->d_side != -1) robot->getConnectorParams(robot->_conn[i]->d_type, robot->_conn[i]->d_side, R, p);

		// PAT to transform mesh
		osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
		transform->setPosition(osg::Vec3d(p[0], p[1], p[2]));
		transform->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));

		// create node to hold mesh
		osg::ref_ptr<osg::Node> node;
		switch (robot->_conn[i]->type) {
			case BIGWHEEL:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/bigwheel.3ds");
				break;
			case BRIDGE:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/bridge.3ds");
				break;
			case CASTER:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/caster.3ds");
				break;
			case CUBE:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/cube.3ds");
				break;
			case FACEPLATE:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/faceplate.3ds");
				break;
			case GRIPPER:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/gripper.3ds");
				break;
			case OMNIDRIVE:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/omnidrive.3ds");
				break;
			case SIMPLE:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/simple.3ds");
				break;
			case SMALLWHEEL:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/smallwheel.3ds");
				break;
			case TINYWHEEL:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/tinywheel.3ds");
				break;
			case WHEEL:
				node = osgDB::readNodeFile(_tex_path + "linkbot/models/tinywheel.3ds");
				transform->setScale(osg::Vec3d(1, robot->_wheel_radius/robot->_tinywheel_radius, robot->_wheel_radius/robot->_tinywheel_radius));
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
		_robots.back()->robot->addChild(transform);
	}

	// set update callback for robot
	//_robots.back()->robot->setUpdateCallback(new linkbotNodeCallback(robot, _robots.back()->led));

	// set tracking
	robot->_trace = tracking;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}

int Graphics::draw(CMobot *robot, int tracking, double *rgb) {
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
	_robots.back()->led = new osg::ShapeDrawable(cyl);
	_robots.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY_R]->addDrawable(_robots.back()->led);

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
		_robots.back()->robot->addChild(pat[i].get());
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
		_robots.back()->robot->addChild(pat);
	}

	// set update callback for robot
	_robots.back()->robot->setUpdateCallback(new mobotNodeCallback(robot, _robots.back()->led));

	// set tracking
	robot->_trace = tracking;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}

int Graphics::draw(CNXT *robot, int tracking, double *rgb) {
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
	_robots.back()->led = new osg::ShapeDrawable(cyl);
	_robots.back()->led->setColor(osg::Vec4(robot->_rgb[0], robot->_rgb[1], robot->_rgb[2], 1));
	body[robot->BODY]->addDrawable(_robots.back()->led);

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
		_robots.back()->robot->addChild(pat[i].get());
	}

	// set update callback for robot
	_robots.back()->robot->setUpdateCallback(new nxtNodeCallback(robot, _robots.back()->led));

	// set tracking
	robot->_trace = tracking;

	// send back rgb
	rgb[0] = robot->_rgb[0];
	rgb[1] = robot->_rgb[1];
	rgb[2] = robot->_rgb[2];
	rgb[3] = 1;

	// success
	return 0;
}

void* Graphics::graphics_thread(void *arg) {
	// cast viewer
	Graphics *sim = (Graphics *)arg;

	// viewer event handlers
	sim->_view->addEventHandler(new osgViewer::WindowSizeHandler);

	// signal connection functions that graphics are set up
	SIGNAL(&(sim->_graphics_cond), &(sim->_graphics_mutex), sim->_graphics = 1);

	// run viewer
	MUTEX_LOCK(&(sim->_viewer_mutex));
	while (sim->_viewer && !sim->_view->done()) {
		MUTEX_UNLOCK(&(sim->_viewer_mutex));

		sim->_view->frame();
		if (sim->_staging->getNumChildren()) {
			sim->_scene->addChild(sim->_staging->getChild(0));
			sim->_staging->removeChild(0, 1);
		}
		if (sim->_ending) {
			sim->_scene->removeChild(sim->_scene->getChild(sim->_ending));
			sim->_ending = 0;
		}

		MUTEX_LOCK(&(sim->_viewer_mutex));
	}
	MUTEX_UNLOCK(&(sim->_viewer_mutex));

	// clean up viewer & root
	sim->_view->setSceneData(NULL);
#ifdef _WIN32_
	delete sim->_view;
#endif

	// trigger end of code when graphics window is closed
	sim->_sim->done();

	// return
	return arg;
}

/**********************************************************
	MoveEarthySkyWithEyePointTransform
 **********************************************************/
bool MoveEarthySkyWithEyePointTransform::computeLocalToWorldMatrix(osg::Matrix &matrix,osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.preMultTranslate(eyePointLocal);
	}
	return true;
}

bool MoveEarthySkyWithEyePointTransform::computeWorldToLocalMatrix(osg::Matrix &matrix,osg::NodeVisitor *nv) const {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
		osg::Vec3 eyePointLocal = cv->getEyeLocal();
		matrix.postMultTranslate(-eyePointLocal);
	}
	return true;
}

/**********************************************************
	TexMatCallback
 **********************************************************/
void TexMatCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osgUtil::CullVisitor *cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
	if (cv) {
		const osg::Matrix& MV = *(cv->getModelViewMatrix());
		const osg::Matrix R = osg::Matrix::rotate( osg::DegreesToRadians(112.0f), 0.0f,0.0f,1.0f)*
								osg::Matrix::rotate( osg::DegreesToRadians(90.0f), 1.0f,0.0f,0.0f);

		osg::Quat q = MV.getRotate();
		const osg::Matrix C = osg::Matrix::rotate( q.inverse() );

		_texMat.setMatrix( C*R );
	}
	traverse(node, nv);
}

/**********************************************************
	Keyboard Event Handler
 **********************************************************/
keyboardEventHandler::keyboardEventHandler(osgText::Text *text) {
	_text = text;
}

void keyboardEventHandler::accept(osgGA::GUIEventHandlerVisitor &v) {
	v.visit(*this);
};

bool keyboardEventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	osg::Group *root = dynamic_cast<osg::Group *>(viewer->getSceneData());
	osgShadow::ShadowedScene *shadow = dynamic_cast<osgShadow::ShadowedScene*>(root->getChild(0));

	switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN:
			switch (ea.getKey()) {
				case '1': {
					osg::Vec3f eye = osg::Vec3f(0.7, -0.7, 0.55);
					osg::Vec3f center = osg::Vec3f(0.1, 0.3, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case '2': {
					osg::Vec3f eye = osg::Vec3f(0, 0, 5);
					osg::Vec3f center = osg::Vec3f(0, 0, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case 'n': {
					osg::Billboard *billboard = dynamic_cast<osg::Billboard *>(root->getChild(3));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					billboard = dynamic_cast<osg::Billboard *>(root->getChild(4));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					return true;
				}
				case 'r': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) { geode->setNodeMask(VISIBLE_MASK); }
							for (int j = 2; j < (int)(shadow->getChild(i)->asGroup()->getNumChildren()); j++) {
								osg::PositionAttitudeTransform *pat;
								pat = dynamic_cast<osg::PositionAttitudeTransform *>(shadow->getChild(i)->asGroup()->getChild(j));
								pat->setNodeMask((pat->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
								//pat->getNodeMask() ? g_sim->setCollisions(1) : g_sim->setCollisions(0);
							}
						}
					}
					return true;
				}
				case 't': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) {
								geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
							}
						}
					}
					return true;
				}
				default:
					//g_sim->setPause(2);
					//if (g_sim->getPause())
					//	_text->setText("Paused: Press any key to restart");
					//else
					//	_text->setText("");
					return true;
			}
		default:
			return false;
	}
}

/**********************************************************
	Picking Event Handler
 **********************************************************/
pickHandler::pickHandler(void) {
	_mx = 0.0;
	_my = 0.0;
}

bool pickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	if (!viewer) return false;

	switch (ea.getEventType()) {
		case(osgGA::GUIEventAdapter::PUSH):
		case(osgGA::GUIEventAdapter::MOVE):
			_mx = ea.getX();
			_my = ea.getY();
			return false;
		case(osgGA::GUIEventAdapter::RELEASE):
			if (_mx == ea.getX() && _my == ea.getY())
				pick(ea, viewer);
			return true;
		default:
			return false;
	}
}

void pickHandler::pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer) {
	osg::Node *scene = viewer->getSceneData();
	if (!scene) return;

	osg::Group *grandparent = 0;
	osgUtil::LineSegmentIntersector *picker;
	double x = ea.getXnormalized(), y = ea.getYnormalized();
	picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, x, y);
	picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE);
	osgUtil::IntersectionVisitor iv(picker);
	iv.setTraversalMask(IS_PICKABLE_MASK);
	viewer->getCamera()->accept(iv);
	//scene->accept(iv);

	if (picker->containsIntersections()) {
		/*std::multiset<osgUtil::LineSegmentIntersector::Intersection> ins = picker->getIntersections();
		std::multiset<osgUtil::LineSegmentIntersector::Intersection>::iterator it = ins.begin();
		osg::NodePath nodePath;
		for(int j = 0; j < ins.size(); j++) {
			nodePath = (*it).nodePath;
			it++;
			printf("%d nodePath:\t", j);
			for(int i = 0; i < nodePath.size(); i++) {
				printf("%s ", nodePath[i]->className());
			}
			printf("\n");
		}*/

		// get node at first intersection
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
		osg::NodePath &nodePath = intersection.nodePath;

		// get robot node
		grandparent = (nodePath.size()>=3) ? dynamic_cast<osg::Group *>(nodePath[nodePath.size()-3]) : 0;

		// toggle HUD
		if (grandparent && (grandparent->getName() == "robot")) {
			osg::Geode *geode = dynamic_cast<osg::Geode *>(grandparent->getChild(0));
			geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
		}
	}
}

/**********************************************************
	Ground Node Callback
 **********************************************************/
groundNodeCallback::groundNodeCallback(Ground *ground) {
	_ground = ground;
}

void groundNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos = dBodyGetPosition(_ground->body);
		const double *quat = dBodyGetQuaternion(_ground->body);
		osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	}
	traverse(node, nv);
}

/**********************************************************
	Linkbot Node Callback
 **********************************************************/
linkbotNodeCallback::linkbotNodeCallback(CLinkbotT *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void linkbotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+4; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw connectors
		for (int j = 0; j < _robot->_conn.size(); j++) {
			dMatrix3 R;
			dQuaternion Q;
			double p[3] = {0};
			if (_robot->_conn[j]->d_side != -1) {
				_robot->getFaceParams(_robot->_conn[j]->face, R, p);
				_robot->getConnectorParams(_robot->_conn[j]->d_type, _robot->_conn[j]->d_side, R, p);
				p[0] += R[0]*_robot->_conn[j]->o[0] + R[1]*_robot->_conn[j]->o[1] + R[2]*_robot->_conn[j]->o[2];
				p[1] += R[4]*_robot->_conn[j]->o[0] + R[5]*_robot->_conn[j]->o[1] + R[6]*_robot->_conn[j]->o[2];
				p[2] += R[8]*_robot->_conn[j]->o[0] + R[9]*_robot->_conn[j]->o[1] + R[10]*_robot->_conn[j]->o[2];
				dRtoQ(R, Q);
			}
			else {
				const double *pos = dBodyGetPosition(_robot->_conn[j]->body);
				p[0] = pos[0]; p[1] = pos[1]; p[2] = pos[2];
				const double *rot = dBodyGetQuaternion(_robot->_conn[j]->body);
				Q[0] = rot[0]; Q[1] = rot[1]; Q[2] = rot[2]; Q[3] = rot[3];
			}
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
	//	if (g_sim->getUnits()) {
	//		sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
	//			_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
	//	}
	//	else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		//}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			geode2->setNodeMask(VISIBLE_MASK);
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			colors->push_back(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0f) );
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// draw led
		_led->setColor(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0));
	}
	traverse(node, nv);
}

/**********************************************************
	Mobot Node Callback
 **********************************************************/
mobotNodeCallback::mobotNodeCallback(CMobot *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void mobotNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+5; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw connectors
		for (int j = 0; j < _robot->_conn.size(); j++) {
			pos = dBodyGetPosition(_robot->_conn[j]->body);
			quat = dBodyGetQuaternion(_robot->_conn[j]->body);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		/*if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {*/
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		//}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			geode2->setNodeMask(VISIBLE_MASK);
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// draw led
		_led->setColor(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0));
	}
	traverse(node, nv);
}

/**********************************************************
	NXT Node Callback
 **********************************************************/
nxtNodeCallback::nxtNodeCallback(CNXT *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void nxtNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+3; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		/*if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {*/
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		//}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			geode2->setNodeMask(VISIBLE_MASK);
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// draw led
		_led->setColor(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0));
	}
	traverse(node, nv);
}

/**********************************************************
	Cubus Node Callback
 **********************************************************/
cubusNodeCallback::cubusNodeCallback(Cubus *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void cubusNodeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, j, k;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+7; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw connectors
		for (j = 0; j < _robot->_conn.size(); j++) {
			dMatrix3 R;
			dQuaternion Q;
			double p[3] = {0};
			_robot->getFaceParams(_robot->_conn[j]->face, R, p);
			if (_robot->_conn[j]->d_side != -1) _robot->getConnectorParams(_robot->_conn[j]->d_type, _robot->_conn[j]->d_side, R, p);
			p[0] += R[0]*_robot->_conn[j]->o[0] + R[1]*_robot->_conn[j]->o[1] + R[2]*_robot->_conn[j]->o[2];
			p[1] += R[4]*_robot->_conn[j]->o[0] + R[5]*_robot->_conn[j]->o[1] + R[6]*_robot->_conn[j]->o[2];
			p[2] += R[8]*_robot->_conn[j]->o[0] + R[9]*_robot->_conn[j]->o[1] + R[10]*_robot->_conn[j]->o[2];
			dRtoQ(R, Q);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i+j));
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));
		}
		// draw sensors
		for (k = 0; k < _robot->_sensor.size(); k++) {
			pos = dGeomGetOffsetPosition(_robot->_sensor[k]->geom);
			quat= dGeomGetOffsetRotation(_robot->_sensor[k]->geom);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i+j+k));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
		/*if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {*/
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		//}
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// draw tracking line
		if (_robot->_trace) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			geode2->setNodeMask(VISIBLE_MASK);
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			colors->push_back(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0f) );
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// draw led
		_led->setColor(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0));
	}
	traverse(node, nv);
}

