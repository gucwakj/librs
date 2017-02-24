#include <algorithm>
#define NOMINMAX 1		// f*$k microsoft

#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/ComputeBoundsVisitor>
#include <osg/Depth>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/TexMat>
#include <osg/TextureCubeMap>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgFX/Outline>
#include <osgGA/OrbitManipulator>
#include <osgShadow/ShadowedScene>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <rs/Enum>
#include <rs/Macros>
#include <rsScene/FixedManipulator>
#include <rsScene/MouseHandler>
#include <rsScene/Scene>
#include <rsScene/SkyTransform>
#include <rsScene/TextureCallback>

#ifdef RS_RESEARCH
#include <rsScene/OpenCVOperation>
#endif

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
	_staging[2] = new osg::Group;

	// set default level to load
	_level = -1;

	// set default grid options
	_units = false;					// customary
	_grid.push_back(rs::IN2M(1));	// 1 inch per tic
	_grid.push_back(rs::IN2M(12));	// 12 inches per hash
	_grid.push_back(rs::IN2M(-48));	// min x
	_grid.push_back(rs::IN2M(48));	// max x
	_grid.push_back(rs::IN2M(-48));	// min y
	_grid.push_back(rs::IN2M(48));	// max y
	_grid.push_back(1);				// enabled?

	// set thread mutex
	RS_MUTEX_INIT(&_thread_mutex);
	_thread = false;
	_osgThread = 0;

	// set texture path
	_tex_path = this->getTexturePath();
	_path.resize(rs::Num_Images);
	_path[rs::Ground].append(_tex_path).append("background/outdoors/ground.3ds");
	_path[rs::Front].append(_tex_path).append("background/outdoors/sky/front.png");
	_path[rs::LeftSide].append(_tex_path).append("background/outdoors/sky/left.png");
	_path[rs::Back].append(_tex_path).append("background/outdoors/sky/back.png");
	_path[rs::RightSide].append(_tex_path).append("background/outdoors/sky/right.png");
	_path[rs::Top].append(_tex_path).append("background/outdoors/sky/top.png");
	_path[rs::Bottom].append(_tex_path).append("background/outdoors/sky/bottom.png");

	// flags for graphical output options
	_highlight = false;
	_label = true;
	_rate = 20;
	_view = Scene::ThirdPerson;
	//_view = Scene::FirstPerson;
	RS_MUTEX_INIT(&_theta_mutex);
}

Scene::~Scene(void) {
	// stop thread
	if (_osgThread) {
		RS_MUTEX_LOCK(&_thread_mutex);
		_thread = false;
		RS_MUTEX_UNLOCK(&_thread_mutex);
		RS_THREAD_JOIN(_osgThread);
		RS_COND_DESTROY(&_graphics_cond);
		RS_MUTEX_DESTROY(&_graphics_mutex);
		// clean mutexes
		RS_MUTEX_DESTROY(&_thread_mutex);
		RS_MUTEX_DESTROY(&_theta_mutex);
	}
}

/**********************************************************
	public functions
 **********************************************************/
void Scene::addAndRemoveChildren(bool clean) {
	// remove objects from scene
	while (_staging[1]->getNumChildren()) {
		_scene->removeChild(_staging[1]->getChild(0));
		_staging[1]->removeChild(0, 1);
	}

	// add new objects to scene
	while (_staging[0]->getNumChildren()) {
		// add child to scene
		_scene->addChild(_staging[0]->getChild(0));
		// attach camera for FirstPerson
		if (_view == Scene::FirstPerson && _staging[0]->getChild(0)->getName() == "robot0") {
			osg::Group *test = NULL;
			for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
				test = dynamic_cast<osg::Group *>(_scene->getChild(i));
				if (test && (!test->getName().compare(0, 5, "robot"))) {
					rsScene::FixedManipulator *manip;
					manip = dynamic_cast<rsScene::FixedManipulator*>(_viewer->getCameraManipulator());
					manip->setNode(test);
					manip->setTrackNode(test);
					manip->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION);
					osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(_staging[0]->getChild(0)->asGroup()->getChild(2 + i));
					manip->setRotation(osg::Quat(0, sin(rs::Pi/4), 0, cos(rs::Pi/4))*pat->getAttitude()*osg::Quat(0, 0, sin(rs::Pi/2), cos(rs::Pi/2)));
					manip->setDistance(0.5);
					break;
				}
			}
		}
		// remove staged child
		_staging[0]->removeChild(0, 1);
	}

	// clean out old background if requested
	if (clean) _background->removeChildren(0, _background->getNumChildren());

	// add new background to scene
	while (_staging[2]->getNumChildren()) {
		_background->addChild(_staging[2]->getChild(0));
		_staging[2]->removeChild(0, 1);
	}

	// share newly created data with other nodes
	osgDB::SharedStateManager *ssm = osgDB::Registry::instance()->getSharedStateManager();
	if (ssm) ssm->share(_root);
}

void Scene::addChildrenToBackground(void) {
	while (_staging[0]->getNumChildren()) {
		_background->addChild(_staging[0]->getChild(0));
		_staging[0]->removeChild(0, 1);
	}
}

void Scene::addHighlight(int id, bool robot, bool preconfig, bool exclusive, const rs::Vec &c) {
	// deselect everything
	if (exclusive) {
		// find nodes of intersection
		osg::Group *test = NULL;
		for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
			test = dynamic_cast<osg::Group *>(_scene->getChild(i));
			// get preconfig node
			if (test && (!test->getName().compare(0, 3, "pre"))) {
				if (!strcmp(test->getChild(0)->asGroup()->getChild(2)->asTransform()->getChild(0)->className(), "Outline"))
					this->toggleHighlight(test, test, c);
			}
			// get robot node
			else if (test && (!test->getName().compare(0, 5, "robot"))) {
				if (!strcmp(test->getChild(2)->asTransform()->getChild(0)->className(), "Outline"))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(2)->asTransform()->getChild(0)), c);
			}
			// get obstacle node - multi
			else if (test && !test->getName().compare(0, 8, "obstacle") && test->getNumChildren() > 1) {
				if (!strcmp(test->getChild(0)->asGroup()->getChild(0)->asTransform()->getChild(0)->className(), "Outline"))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(0)->asGroup()->getChild(0)->asTransform()->getChild(0)), c);
			}
			// get obstacle node - single
			else if (test && !test->getName().compare(0, 8, "obstacle") && test->getNumChildren() == 1) {
				if (!strcmp(test->getChild(0)->asTransform()->getChild(0)->className(),"Outline"))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(0)->asTransform()->getChild(0)), c);
			}
			// get marker node
			else if (test && !test->getName().compare(0, 6, "marker")) {
				if (!strcmp(test->getChild(0)->asTransform()->getChild(0)->className(), "Outline"))
					this->toggleHighlight(test, dynamic_cast<osg::Node *>(test->getChild(0)->asTransform()->getChild(0)), c);
			}
		}
	}

	// set highlight of item
	osg::Group *test = NULL;
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		// get preconfig node
		if (robot && test && !test->getName().compare(std::string("pre").append(std::to_string(id)))) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (this->intersect_new_item(test->getName(), cbbv.getBoundingBox())) {
				this->setHUD(true);
				this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
				this->getHUDText()->setText("Objects are Possibly Colliding!");
				this->toggleHighlight(test, test, rs::Vec(1, 0, 0), true);
			}
			else {
				this->setHUD(false);
				this->toggleHighlight(test, test, c);
			}
		}
		// get robot node
		else if (robot && test && !test->getName().compare(std::string("robot").append(std::to_string(id)))) {
			if (!preconfig) {
				osg::ComputeBoundsVisitor cbbv;
				test->accept(cbbv);
				if (this->intersect_new_item(test->getName(), cbbv.getBoundingBox())) {
					this->setHUD(true);
					this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
					this->getHUDText()->setText("Objects are Possibly Colliding!");
					this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), rs::Vec(1, 0, 0), true);
				}
				else {
					this->setHUD(false);
					this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), c);
				}
			}
			else {
				this->setHUD(false);
				this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), c, true);
			}
		}
		// get obstacle node
		else if (!robot && test && !test->getName().compare(std::string("obstacle").append(std::to_string(id)))) {
			if (test->getNumChildren() > 1) {
				bool free = true;
				osg::Group *test2;
				for (unsigned int j = 0; j < test->getNumChildren(); j++) {
					test2 = test->getChild(j)->asGroup();
					osg::ComputeBoundsVisitor cbbv;
					test2->accept(cbbv);
					if (this->intersect_new_item(test->getName(), cbbv.getBoundingBox())) {
						free = false;
						break;
					}
				}
				if (free) {
					this->setHUD(false);
					this->toggleHighlight(test, test->getChild(0)->asGroup()->getChild(0)->asTransform()->getChild(0), c);
				}
				else {
					this->setHUD(true);
					this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
					this->getHUDText()->setText("Objects are Possibly Colliding!");
					this->toggleHighlight(test, test->getChild(0)->asGroup()->getChild(0)->asTransform()->getChild(0), rs::Vec(1, 0, 0));
				}
			}
			else {
				osg::ComputeBoundsVisitor cbbv;
				test->accept(cbbv);
				if (this->intersect_new_item(test->getName(), cbbv.getBoundingBox())) {
					this->setHUD(true);
					this->getHUDText()->setColor(osg::Vec4(1, 0, 0, 1));
					this->getHUDText()->setText("Objects are Possibly Colliding!");
					this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), rs::Vec(1, 0, 0));
				}
				else {
					this->setHUD(false);
					this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), c);
				}
			}
		}
		// get marker node
		else if (!robot && test && !test->getName().compare(std::string("marker").append(std::to_string(id)))) {
			this->setHUD(false);
			this->toggleHighlight(test, test->getChild(0)->asTransform()->getChild(0), c);
		}
	}
}

Group* Scene::createPreconfig(int id) {
	// create new robot
	osg::ref_ptr<osg::Group> group = new osg::Group();
	group->ref();

	// set user properties of node
	group->setName(std::string("pre").append(std::to_string(id)));

	// return robot group
	return group.get();
}

Group* Scene::createRobot(rsScene::Robot *robot) {
	// create new robot
	osg::ref_ptr<osg::Group> group = new osg::Group();
	group->ref();

	// send data to robot
	robot->setModelPath(this->getTexturePath());
	robot->setUnits(_units);

	// return robot group
	return group.get();
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

int Scene::deletePreconfig(int id) {
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		osg::Group *test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		if (test && !test->getName().compare(std::string("pre").append(std::to_string(id)))) {
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

int Scene::drawMarker(int id, int type, const rs::Pos &p1, const rs::Pos &p2, const rs::Pos &pt, const rs::Vec &c, int size, std::string s) {
	// create geode
	osg::ref_ptr<osg::Group> marker = new osg::Group();
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	// draw specific marker
	switch (type) {
		case rs::Arc: {
			float start = p2[0];
			float end = p2[1];
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(start) - p1[0], p1[1] + p1[2]*sin(start) - p1[1], 0.001));
			int n = 50;
			for (int i = 1; i < n; i++) {
				double rad = i*((end-start)/n);
				double x = p1[0] + p1[2]*cos(start + rad);
				double y = p1[1] + p1[2]*sin(start + rad);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(end) - p1[0], p1[1] + p1[2]*sin(end) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::ArcSector: {
			float start = p2[0];
			float end = p2[1];
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0.001));
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(start) - p1[0], p1[1] + p1[2]*sin(start) - p1[1], 0.001));
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(start) - p1[0], p1[1] + p1[2]*sin(start) - p1[1], 0.001));
			int n = 50;
			for (int i = 1; i < n; i++) {
				double rad = i*((end-start)/n);
				double x = p1[0] + p1[2]*cos(start + rad);
				double y = p1[1] + p1[2]*sin(start + rad);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(end) - p1[0], p1[1] + p1[2]*sin(end) - p1[1], 0.001));
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(end) - p1[0], p1[1] + p1[2]*sin(end) - p1[1], 0.001));
			vert->push_back(osg::Vec3(0, 0, 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::ArcSegment: {
			float start = p2[0];
			float end = p2[1];
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(start) - p1[0], p1[1] + p1[2]*sin(start) - p1[1], 0.001));
			int n = 50;
			for (int i = 1; i < n; i++) {
				double rad = i*((end-start)/n);
				double x = p1[0] + p1[2]*cos(start + rad);
				double y = p1[1] + p1[2]*sin(start + rad);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(end) - p1[0], p1[1] + p1[2]*sin(end) - p1[1], 0.001));
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(end) - p1[0], p1[1] + p1[2]*sin(end) - p1[1], 0.001));
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(start) - p1[0], p1[1] + p1[2]*sin(start) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Arrow: {
			float length = sqrt((p2[0] - p1[0])*(p2[0] - p1[0]) + (p2[1] - p1[1])*(p2[1] - p1[1]));
			float angle = asin((p2[1] - p1[1])/length);
			float bit = 0.25*length;
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(p2[0] + bit*cos(2.356 + angle) - p1[0], p2[1] + bit*sin(2.356 + angle) - p1[1], 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(p2[0] + bit*cos(3.927 + angle) - p1[0], p2[1] + bit*sin(3.927 + angle) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Circle: {
			{ // fill
				osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
				osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
				osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
				colors->push_back(osg::Vec4(fill[0], fill[1], fill[2], fill[3]));
				geom->setColorArray(colors.get());
				geom->setColorBinding(osg::Geometry::BIND_OVERALL);
				int num = 50;
				double angle = 2*rs::Pi/num, angle1 = 0.0;
				vert->push_back(osg::Vec3(p1[2]*cos(angle1), p1[2]*sin(angle1), 0.001));
				for (int i = 0; i < num; i++) {
					vert->push_back(osg::Vec3(p1[2]*cos(angle1), p1[2]*sin(angle1), 0.001));
					angle1 += angle;
				}
				geom->setVertexArray(vert.get());
				geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, num));
				geom->getOrCreateStateSet()->setMode(GL_FILL, osg::StateAttribute::ON);
				geode->addDrawable(geom.get());
			}
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(0) - p1[0], p1[1] + p1[2]*sin(0) - p1[1], 0.001));
			int n = 50;
			for (int i = 1; i < n; i++) {
				double rad = i*(2*rs::Pi/n);
				double x = p1[0] + p1[2]*cos(rad);
				double y = p1[1] + p1[2]*sin(rad);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + p1[2]*cos(0) - p1[0], p1[1] + p1[2]*sin(0) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Dot: {
			geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), size/500.0)));
			break;
		}
		case rs::Ellipse: {
			float w = p2[0]/2;
			float h = p2[1]/2;
			float a = p2[2];
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(p1[0] + w*cos(0)*cos(a) - p1[0], p1[1] + w*cos(0)*sin(a) + h*sin(0)*cos(a) - p1[1], 0.001));
			int n = 50;
			for (int i = 1; i < n; i++) {
				double rad = i*(2*rs::Pi/n);
				double x = p1[0] + w*cos(rad)*cos(a) - h*sin(rad)*sin(a);
				double y = p1[1] + w*cos(rad)*sin(a) + h*sin(rad)*cos(a);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + w*cos(0)*cos(a) - p1[0], p1[1] + w*cos(0)*sin(a) + h*sin(0)*cos(a) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Line: {
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Polygon: {
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			int n = rs::M2IN(p2[0] + 0.001);
			double r = p1[2]*sin(rs::Pi/n);
			vert->push_back(osg::Vec3(p1[0] + r*cos(0) - p1[0], p1[1] + r*sin(0) - p1[1], 0.001));
			for (int i = 1; i < n; i++) {
				double rad = i*(2*rs::Pi/n);
				double x = p1[0] + r*cos(rad);
				double y = p1[1] + r*sin(rad);
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
				vert->push_back(osg::Vec3(x - p1[0], y - p1[1], 0.001));
			}
			vert->push_back(osg::Vec3(p1[0] + r*cos(0) - p1[0], p1[1] + r*sin(0) - p1[1], 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2 + 2*(n-1)));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Quad: {
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0.001));
			vert->push_back(osg::Vec3(p1[1] - p1[0], p2[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[1] - p1[0], p2[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[2] - p1[0], p2[2] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[2] - p1[0], p2[2] - p2[0], 0.001));
			vert->push_back(osg::Vec3(pt[0] - p1[0], pt[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(pt[0] - p1[0], pt[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(0, 0, 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 8));
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->addDrawable(geom.get());
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Rectangle: {
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], 0, 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], 0, 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(p2[0] - p1[0], p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(0, p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(0, p2[1] - p1[1], 0.001));
			vert->push_back(osg::Vec3(0, 0, 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 8));
			geode->addDrawable(geom.get());
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
		case rs::Text: {
			osg::ref_ptr<osgText::Text> label = new osgText::Text();
			label->setAlignment(osgText::Text::CENTER_CENTER);
			label->setAxisAlignment(osgText::Text::SCREEN);
			label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
			label->setCharacterSize(size);
			label->setDrawMode(osgText::Text::TEXT);
			label->setPosition(osg::Vec3(0, 0, 0));
			label->setText(s);
			geode->addDrawable(label.get());
			break;
		}
		case rs::Triangle: {
			osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
			osg::ref_ptr<osg::Vec3Array> vert = new osg::Vec3Array();
			vert->push_back(osg::Vec3(0, 0, 0.001));
			vert->push_back(osg::Vec3(p1[1] - p1[0], p2[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[1] - p1[0], p2[1] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[2] - p1[0], p2[2] - p2[0], 0.001));
			vert->push_back(osg::Vec3(p1[2] - p1[0], p2[2] - p2[0], 0.001));
			vert->push_back(osg::Vec3(0, 0, 0.001));
			geom->setVertexArray(vert.get());
			geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));
			geode->addDrawable(geom.get());
			osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
			geom->setColorArray(colors.get());
			geom->setColorBinding(osg::Geometry::BIND_OVERALL);
			osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
			width->setWidth(3*size);
			geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);
			break;
		}
	}

	// set rendering properties
	geode->getOrCreateStateSet()->setRenderBinDetails(11, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	geode->setCullingActive(false);

	// add positioning capability
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();
	switch (type) {
		case rs::Arc:
		case rs::ArcSector:
		case rs::ArcSegment:
		case rs::Arrow:
		case rs::Circle:
		case rs::Ellipse:
		case rs::Polygon:
		case rs::Rectangle:
			pat->setPosition(osg::Vec3d(p1[0], p1[1], 0));
			break;
		case rs::Quad:
			pat->setPosition(osg::Vec3d(p1[0], p2[0], 0));
			break;
		case rs::Triangle:
			pat->setPosition(osg::Vec3d(p1[0], p2[0], 0));
			break;
		default:
			pat->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
			break;
	}
	pat->setAttitude(osg::Quat(0, 0, 0, 1));
	pat->addChild(geode.get());
	marker->addChild(pat.get());

	// set user properties of node
	marker->setName(std::string("marker").append(std::to_string(id)));

	// add to scenegraph
	_staging[0]->addChild(marker.get());

	return 0;
}

Obstacle* Scene::drawObstacle(int id, int type, const rs::Pos &p, const rs::Vec &c, const rs::Vec &l, const rs::Quat &q) {
	// create obstacle objects
	osg::ref_ptr<osg::Group> obstacle = new osg::Group();
	osg::ref_ptr<osg::Geode> body = new osg::Geode();
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();

	switch (type) {
		case rs::Box:
			// create body
			body->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0, 0, 0), l[0], l[1], l[2])));
			// add to pat
			pat->addChild(body.get());
			// position
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			// add to obstacle
			obstacle->addChild(pat.get());
			break;
		case rs::CompetitionBorder: {
			// front
			osg::ref_ptr<osg::Geode> body0 = new osg::Geode();
			body0->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[2], l[0])));
			body0->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body0->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body0->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body0->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body0->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat0 = new osg::PositionAttitudeTransform();
			pat0->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(0, -l[1]/2, 0.04) + osg::Vec3d(p[0], p[1], p[2]));
			pat0->setAttitude(osg::Quat(0, 0.707107, 0, 0.707107) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat0->addChild(body0.get());
			osg::ref_ptr<osg::Group> group0 = new osg::Group();
			group0->addChild(pat0.get());
			// back
			osg::ref_ptr<osg::Geode> body1 = new osg::Geode();
			body1->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[2], l[0])));
			body1->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body1->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body1->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body1->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body1->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat1 = new osg::PositionAttitudeTransform();
			pat1->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(0, l[1]/2, 0.04) + osg::Vec3d(p[0], p[1], p[2]));
			pat1->setAttitude(osg::Quat(0, 0.707107, 0, 0.707107) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat1->addChild(body1.get());
			osg::ref_ptr<osg::Group> group1 = new osg::Group();
			group1->addChild(pat1.get());
			// left
			osg::ref_ptr<osg::Geode> body2 = new osg::Geode();
			body2->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[2], l[1])));
			body2->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body2->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body2->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body2->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body2->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat2 = new osg::PositionAttitudeTransform();
			pat2->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(-l[0]/2, 0, 0.04) + osg::Vec3d(p[0], p[1], p[2]));
			pat2->setAttitude(osg::Quat(0.707107, 0, 0, 0.707107) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat2->addChild(body2.get());
			osg::ref_ptr<osg::Group> group2 = new osg::Group();
			group2->addChild(pat2.get());
			// right
			osg::ref_ptr<osg::Geode> body3 = new osg::Geode();
			body3->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[2], l[1])));
			body3->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body3->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body3->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body3->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body3->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat3 = new osg::PositionAttitudeTransform();
			pat3->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(l[0]/2, 0, 0.04) + osg::Vec3d(p[0], p[1], p[2]));
			pat3->setAttitude(osg::Quat(0.707107, 0, 0, 0.707107) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat3->addChild(body3.get());
			osg::ref_ptr<osg::Group> group3 = new osg::Group();
			group3->addChild(pat3.get());
			// corner
			osg::ref_ptr<osg::Geode> body4 = new osg::Geode();
			body4->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.02, 0.04)));
			body4->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body4->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body4->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body4->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body4->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat4 = new osg::PositionAttitudeTransform();
			pat4->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(-l[0]/2, -l[1]/2, 0.02) + osg::Vec3d(p[0], p[1], p[2]));
			pat4->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			pat4->addChild(body4.get());
			osg::ref_ptr<osg::Group> group4 = new osg::Group();
			group4->addChild(pat4.get());
			// corner
			osg::ref_ptr<osg::Geode> body5 = new osg::Geode();
			body5->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.02, 0.04)));
			body5->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body5->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body5->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body5->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body5->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat5 = new osg::PositionAttitudeTransform();
			pat5->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(-l[0]/2, l[1]/2, 0.02) + osg::Vec3d(p[0], p[1], p[2]));
			pat5->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			pat5->addChild(body5.get());
			osg::ref_ptr<osg::Group> group5 = new osg::Group();
			group5->addChild(pat5.get());
			// corner
			osg::ref_ptr<osg::Geode> body6 = new osg::Geode();
			body6->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.02, 0.04)));
			body6->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body6->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body6->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body6->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body6->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat6 = new osg::PositionAttitudeTransform();
			pat6->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(l[0]/2, l[1]/2, 0.02) + osg::Vec3d(p[0], p[1], p[2]));
			pat6->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			pat6->addChild(body6.get());
			osg::ref_ptr<osg::Group> group6 = new osg::Group();
			group6->addChild(pat6.get());
			// corner
			osg::ref_ptr<osg::Geode> body7 = new osg::Geode();
			body7->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.02, 0.04)));
			body7->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body7->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body7->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body7->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body7->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat7 = new osg::PositionAttitudeTransform();
			pat7->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(l[0]/2, -l[1]/2, 0.02) + osg::Vec3d(p[0], p[1], p[2]));
			pat7->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			pat7->addChild(body7.get());
			osg::ref_ptr<osg::Group> group7 = new osg::Group();
			group7->addChild(pat7.get());
			// add to body
			obstacle->addChild(group7.get());
			obstacle->addChild(group6.get());
			obstacle->addChild(group5.get());
			obstacle->addChild(group4.get());
			obstacle->addChild(group3.get());
			obstacle->addChild(group2.get());
			obstacle->addChild(group1.get());
			obstacle->addChild(group0.get());
			break;
		}
		case rs::Cylinder:
			// create body
			body->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), l[0], l[1])));
			// add to pat
			pat->addChild(body.get());
			// position
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			// add to obstacle
			obstacle->addChild(pat.get());
			break;
		case rs::HackySack: {
			// create body
			body->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), 0.5*l[0])));
			// create texture object
			osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "obstacles/hackysack.png"));
			tex->setDataVariance(osg::Object::STATIC);
			tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
			tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
			tex->setUnRefImageDataAfterApply(true);
			tex->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
			tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
			tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
			// state set
			body->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
			body->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
			body->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL), osg::StateAttribute::ON);
			// add to pat
			pat->addChild(body.get());
			// position
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			// add to obstacle
			obstacle->addChild(pat.get());
			break;
		}
		case rs::PullupBar: {
			// back left
			osg::ref_ptr<osg::Geode> body0 = new osg::Geode();
			body0->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.0125, 0.16)));
			body0->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body0->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body0->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body0->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body0->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat0 = new osg::PositionAttitudeTransform();
			pat0->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(-0.056569, -0.08, 0.056569) + osg::Vec3d(p[0], p[1], p[2]));
			pat0->setAttitude(osg::Quat(0, 0.382683, 0, 0.923880) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat0->addChild(body0.get());
			osg::ref_ptr<osg::Group> group0 = new osg::Group();
			group0->addChild(pat0.get());
			// back right
			osg::ref_ptr<osg::Geode> body1 = new osg::Geode();
			body1->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.0125, 0.16)));
			body1->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body1->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body1->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body1->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body1->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat1 = new osg::PositionAttitudeTransform();
			pat1->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(0.056569, -0.08, 0.056569) + osg::Vec3d(p[0], p[1], p[2]));
			pat1->setAttitude(osg::Quat(0, -0.382683, 0, 0.923880) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat1->addChild(body1.get());
			osg::ref_ptr<osg::Group> group1 = new osg::Group();
			group1->addChild(pat1.get());
			// front left
			osg::ref_ptr<osg::Geode> body2 = new osg::Geode();
			body2->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.0125, 0.16)));
			body2->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body2->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body2->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body2->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body2->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat2 = new osg::PositionAttitudeTransform();
			pat2->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(-0.056569, 0.08, 0.056569) + osg::Vec3d(p[0], p[1], p[2]));
			pat2->setAttitude(osg::Quat(0, 0.382683, 0, 0.923880) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat2->addChild(body2.get());
			osg::ref_ptr<osg::Group> group2 = new osg::Group();
			group2->addChild(pat2.get());
			// front right
			osg::ref_ptr<osg::Geode> body3 = new osg::Geode();
			body3->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.0125, 0.16)));
			body3->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body3->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body3->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body3->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body3->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat3 = new osg::PositionAttitudeTransform();
			pat3->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(0.056569, 0.08, 0.056569) + osg::Vec3d(p[0], p[1], p[2]));
			pat3->setAttitude(osg::Quat(0, -0.382683, 0, 0.923880) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat3->addChild(body3.get());
			osg::ref_ptr<osg::Group> group3 = new osg::Group();
			group3->addChild(pat3.get());
			// top
			osg::ref_ptr<osg::Geode> body4 = new osg::Geode();
			body4->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3d(0, 0, 0), 0.0125, 0.16)));
			body4->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
			body4->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
			body4->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
			body4->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
			body4->setCullingActive(false);
			osg::ref_ptr<osg::PositionAttitudeTransform> pat4 = new osg::PositionAttitudeTransform();
			pat4->setPosition(osg::Quat(q[0], q[1], q[2], q[3]) * osg::Vec3d(0, 0, 0.113137) + osg::Vec3d(p[0], p[1], p[2]));
			pat4->setAttitude(osg::Quat(0.707107, 0, 0, 0.707107) * osg::Quat(q[0], q[1], q[2], q[3]));
			pat4->addChild(body4.get());
			osg::ref_ptr<osg::Group> group4 = new osg::Group();
			group4->addChild(pat4.get());
			// add to body
			obstacle->addChild(group0.get());
			obstacle->addChild(group1.get());
			obstacle->addChild(group2.get());
			obstacle->addChild(group3.get());
			obstacle->addChild(group4.get());
			break;
		}
		case rs::Sphere:
			// create body
			body->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(0, 0, 0), l[0])));
			// add to pat
			pat->addChild(body.get());
			// position
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			// add to obstacle
			obstacle->addChild(pat.get());
			break;
		case rs::WoodBlock: {
			// create body
			body->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0, 0, 0), l[0], l[1], l[2])));
			// create texture object
			osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_tex_path + "obstacles/wood.png"));
			tex->setDataVariance(osg::Object::STATIC);
			tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
			tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
			tex->setUnRefImageDataAfterApply(true);
			tex->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
			tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
			tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
			// state set
			body->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
			body->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
			body->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL), osg::StateAttribute::ON);
			// add to pat
			pat->addChild(body.get());
			// position
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));
			// add to obstacle
			obstacle->addChild(pat.get());
			break;
		}
	}

	// set rendering properties
	body->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	body->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	body->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(c[0], c[1], c[2], c[3])));
	body->setCullingActive(false);

	// set user properties of node
	obstacle->setName(std::string("obstacle").append(std::to_string(id)));

	// add to scenegraph
	_staging[0]->addChild(obstacle.get());

	// success
	return obstacle.get();
}

void Scene::drawPath(std::vector<double> *xpts, std::vector<double> *ypts) {
	// create line geometry
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	geom->setDataVariance(osg::Object::DYNAMIC);
	geom->setUseDisplayList(false);
	geom->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2*xpts->size()-2));

	// set vertices
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(xpts->at(0), ypts->at(0), 0));
	for (unsigned int i = 1; i < xpts->size()-2; i++) {
		vertices->push_back(osg::Vec3(xpts->at(i), ypts->at(i), 0));
		vertices->push_back(osg::Vec3(xpts->at(i), ypts->at(i), 0));
	}
	vertices->push_back(osg::Vec3(xpts->at(xpts->size()-1), ypts->at(ypts->size()-1), 0));
	geom->setVertexArray(vertices.get());

	// set color
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	colors->push_back(osg::Vec4(1, 0, 0, 1));
	geom->setColorArray(colors.get());
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	// set width
	osg::ref_ptr<osg::LineWidth> width = new osg::LineWidth();
	width->setWidth(50);

	// set rendering properties
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->getOrCreateStateSet()->setRenderBinDetails(200, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setAttributeAndModes(width.get(), osg::StateAttribute::ON);

	// set geode properties
	geode->addDrawable(geom.get());
	geode->setNodeMask(VISIBLE_MASK);
	geode->setName("path");

	// add to scenegraph
	_staging[0]->addChild(geode.get());
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

osgText::Text* Scene::getClockText(void) {
	// get text geode
	osg::Geode *geode = NULL;
	for (unsigned int i = 0; i < _root->getNumChildren(); i++) {
		if (!_root->getChild(i)->getName().compare("ClockProjection")) {
			geode = _root->getChild(i)->asGroup()->getChild(0)->asTransform()->getChild(0)->asGeode();
			return dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		}
	}
	// return
	return NULL;
}

std::string Scene::getTexturePath(void) {
	std::string path;
#ifdef RS_WIN32
	DWORD size = 128;
	HKEY key;
#if defined(RS_WIN64)
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
	osgDB::setLibraryFilePathList("/usr/local/ch/package/chrobosim/bin/");
	path = "/usr/local/ch/package/chrobosim/data/";
#endif
	return path;
}

double Scene::getTheta(void) {
	double theta = 0;
	RS_MUTEX_LOCK(&(_theta_mutex));
	theta = _theta;
	RS_MUTEX_UNLOCK(&(_theta_mutex));
	return theta;
}

void Scene::reidRobot(int id, int num) {
	osg::Group *test = NULL;
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		// preconfig robot
		if (!test->getName().compare(std::string("pre").append(std::to_string(id)))) {
			test->setName(std::string("pre").append(std::to_string(id - num)));
		}
		// individual robot
		if (!test->getName().compare(std::string("robot").append(std::to_string(id)))) {
			test->setName(std::string("robot").append(std::to_string(id - num)));
		}
	}
}

void Scene::setBackgroundImage(int pos, std::string path) {
	if (!path.empty()) _path[pos] = path;
}

void Scene::setClock(bool enable) {
	for (unsigned int i = 0; i < _root->getNumChildren(); i++) {
		if (!_root->getChild(i)->getName().compare("ClockProjection")) {
			if (enable)
				_root->getChild(i)->setNodeMask(VISIBLE_MASK);
			else
				_root->getChild(i)->setNodeMask(NOT_VISIBLE_MASK);
			break;
		}
	}
}

void Scene::setFrameRate(int rate) {
	_rate = (rate > 0) ? rate : _rate;
}

void Scene::setGrid(std::vector<float> grid, bool draw) {
	// save new values
	if (grid[0] != -1) _grid[0] = grid[0];
	if (grid[1] != -1) _grid[1] = grid[1];
	if (grid[2] != -1) _grid[2] = grid[2];
	if (grid[3] != -1) _grid[3] = grid[3];
	if (grid[4] != -1) _grid[4] = grid[4];
	if (grid[5] != -1) _grid[5] = grid[5];
	if (grid[6] != -1) _grid[6] = grid[6];

	// lock thread from playing with _background
	RS_MUTEX_LOCK(&(_thread_mutex));

	// remove old grid
	for (unsigned int i = 0; i < _background->getNumChildren(); i++) {
		osg::Group *test = dynamic_cast<osg::Group *>(_background->getChild(i));
		if (test && !test->getName().compare(std::string("grid")))
			_background->removeChild(test);
	}

	// draw grid if there is a background on which to draw
	if (_level != rs::Level::None && static_cast<int>(_grid[6]) && draw && _view != Scene::FirstPerson) {
		this->draw_grid(_grid[0], _grid[1], _grid[2], _grid[3], _grid[4], _grid[5], _grid[6]);
	}

	// unlock for thread drawing
	RS_MUTEX_UNLOCK(&(_thread_mutex));
}

void Scene::setHighlight(bool highlight) {
	_highlight = highlight;
}

void Scene::setHUD(bool enable) {
	if (_view == Scene::FirstPerson) return;

	// change view
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
	// set new level
	_level = level;

	// level == NONE, return
	if (_level == rs::Level::None) return;

	// lock threaded add/remove
	RS_MUTEX_LOCK(&(_thread_mutex));

	// draw new level
	switch (level) {
		case rs::Level::ActivityMat:
			this->draw_board(0.68, 0.37, 0.68, 0.37);
			break;
		case rs::Level::Board:
			this->draw_board(1.219, 0.610, 1.219, 0.610);
			break;
		case rs::Level::Outdoors:
			this->draw_skybox();
			this->draw_ground();
			break;
		case rs::Level::RPC2014:
			this->draw_board(1.372, 0.762, rs::IN2M(6), rs::IN2M(6));
			break;
		case rs::Level::RPC2015:
			this->draw_board(1.2192, 0.6858, rs::IN2M(6), rs::IN2M(6));
			break;
		case rs::Level::RPC2016:
			this->draw_board(1.2192, 0.6858, rs::IN2M(6), rs::IN2M(6));
			break;
	}

	// unlock threaded add/remove
	RS_MUTEX_UNLOCK(&(_thread_mutex));
}

void Scene::setMouseHandler(rsScene::MouseHandler *mh) {
	// find and remove old handler
	osgViewer::View::EventHandlers eh = _viewer->getEventHandlers();
	for (osgViewer::View::EventHandlers::iterator it = eh.begin(); it != eh.end(); it++) {
		if (!(*it)->getName().compare(0, 5, "mouse")) {
			_viewer->removeEventHandler((*it).get());
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

void Scene::setRobotCallback(osg::Group *robot, osg::NodeCallback *nc) {
	robot->setUpdateCallback(nc);
}

void Scene::setTheta(double theta) {
	RS_MUTEX_LOCK(&(_theta_mutex));
	_theta = theta;
	RS_MUTEX_UNLOCK(&(_theta_mutex));
}

void Scene::setUnits(bool units) {
	_units = units;
}

int Scene::setupCamera(osg::GraphicsContext *gc, double w, double h) {
	// camera properties
	_camera = new osg::Camera();
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
	_camera->getOrCreateStateSet()->setGlobalDefaults();
	_viewer->addSlave(_camera);

	// camera manipulator
	if (_view == Scene::ThirdPerson) {
		osg::ref_ptr<osgGA::OrbitManipulator> cameraManipulator = new osgGA::OrbitManipulator();
		cameraManipulator->setDistance(0.1);
		cameraManipulator->setAllowThrow(false);
		cameraManipulator->setWheelZoomFactor(0);
		cameraManipulator->setVerticalAxisFixed(true);
		cameraManipulator->setElevation(0.5);
		_viewer->setCameraManipulator(cameraManipulator.get());
		_viewer->getCameraManipulator()->setHomePosition(osg::Vec3f(0.6, -0.8, 0.5), osg::Vec3f(0.1, 0.3, 0), osg::Vec3f(0, 0, 1));
	}
	else if (_view == Scene::FirstPerson) {
		osg::ref_ptr<rsScene::FixedManipulator> cameraManipulator = new rsScene::FixedManipulator();
		_viewer->setCameraManipulator(cameraManipulator.get());
	}

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
	osg::ref_ptr<osg::LightSource> ls = new osg::LightSource();
	osg::ref_ptr<osg::Light> myLight = new osg::Light();
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
	this->draw_hud(w, h, pause);

	// draw clock HUD
	this->draw_clock(w, h);

	// draw background pieces for levels
	_background = new osg::Group();
	_background->setName("background");
	_scene->addChild(_background);

	// event handler
	_viewer->addEventHandler(dynamic_cast<KeyboardHandler*>(this));

	// create and add mouse handler
	osg::ref_ptr<MouseHandler> mh = new MouseHandler(this);
	_viewer->addEventHandler(mh.get());

#ifdef RS_RESEARCH
	// create and add screen capture handler
	if (_view == Scene::FirstPerson) {
		_sch = new osgViewer::ScreenCaptureHandler(new rsScene::OpenCVOperation(this));
	}
#endif

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

void Scene::stageChild(osg::Group *group) {
	_staging[0]->addChild(group);
}

void Scene::start(int pause) {
	// init graphics variables
	RS_COND_INIT(&_graphics_cond);
	RS_MUTEX_INIT(&_graphics_mutex);
	_graphics = false;

	// create thread
	RS_THREAD_CREATE(&_osgThread, (void* (*)(void *))&Scene::graphics_thread, (void *)this);

	// wait for graphics to be set up
	RS_MUTEX_LOCK(&_graphics_mutex);
	while (!_graphics) {
		RS_COND_WAIT(&_graphics_cond, &_graphics_mutex);
	}
	RS_MUTEX_UNLOCK(&_graphics_mutex);
}

void Scene::toggleHighlight(osg::Group *parent, osg::Node *child, const rs::Vec &c, bool on) {
	if (!_highlight) return;

	if (!parent->getName().compare(0, 3, "pre")) {
		for (unsigned int i = 0; i < parent->getNumChildren(); i++) {
			this->toggleHighlight(parent->getChild(i)->asGroup(), parent->getChild(i)->asGroup()->getChild(2)->asTransform()->getChild(0), c, on);
		}
	}
	else if (!parent->getName().compare(0, 5, "robot")) {
		// not highlighted yet, do that now
		if (!(dynamic_cast<osgFX::Outline *>(child))) {
			for (unsigned int i = 2; i < parent->getNumChildren(); i++) {
				osg::ref_ptr<osgFX::Outline> outline = new osgFX::Outline();
				outline->setWidth(20);
				outline->setColor(osg::Vec4(c[0], c[1], c[2], 1.0));
				outline->getOrCreateStateSet()->setRenderBinDetails(90, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
				outline->addChild(parent->getChild(i)->asTransform()->getChild(0));
				parent->getChild(i)->asTransform()->replaceChild(parent->getChild(i)->asTransform()->getChild(0), outline.get());
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
		if (parent->getNumChildren() > 1) {
			// not highlighted yet, do that now
			if (!(dynamic_cast<osgFX::Outline *>(child))) {
				for (unsigned int i = 0; i < parent->getNumChildren(); i++) {
					osg::ref_ptr<osgFX::Outline> outline = new osgFX::Outline();
					outline->setWidth(20);
					outline->setColor(osg::Vec4(c[0], c[1], c[2], 1.0));
					outline->getOrCreateStateSet()->setRenderBinDetails(90, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
					outline->addChild(parent->getChild(i)->asGroup()->getChild(0)->asTransform()->getChild(0));
					parent->getChild(i)->asGroup()->getChild(0)->asTransform()->replaceChild(parent->getChild(i)->asGroup()->getChild(0)->asTransform()->getChild(0), outline.get());
				}
			}
			// already highlighted, take it away
			else if (!on) {
				for (unsigned int i = 0; i < parent->getNumChildren(); i++) {
					osgFX::Outline *parentOutline = dynamic_cast<osgFX::Outline *>(parent->getChild(i)->asGroup()->getChild(0)->asTransform()->getChild(0));
					parent->getChild(i)->asGroup()->getChild(0)->asTransform()->replaceChild(parentOutline, parentOutline->getChild(0));
				}
			}
		}
		else {
			// not highlighted yet, do that now
			if (!(dynamic_cast<osgFX::Outline *>(child))) {
				osg::ref_ptr<osgFX::Outline> outline = new osgFX::Outline();
				outline->setWidth(20);
				outline->setColor(osg::Vec4(c[0], c[1], c[2], 1.0));
				outline->getOrCreateStateSet()->setRenderBinDetails(90, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
				outline->addChild(parent->getChild(0)->asTransform()->getChild(0));
				parent->getChild(0)->asTransform()->replaceChild(parent->getChild(0)->asTransform()->getChild(0), outline.get());
			}
			// already highlighted, take it away
			else if (!on) {
				osgFX::Outline *parentOutline = dynamic_cast<osgFX::Outline *>(parent->getChild(0)->asTransform()->getChild(0));
				parent->getChild(0)->asTransform()->replaceChild(parentOutline, parentOutline->getChild(0));
			}
		}
	}
}

void Scene::toggleLabel(osg::Group *parent, osg::Node *child) {
	if (!_label) return;

	if (!parent->getName().compare(0, 5, "robot")) {
		osg::Geode *geode = dynamic_cast<osg::Geode *>(parent->getChild(0));
		geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
	}
	else if (!parent->getName().compare(0, 8, "obstacle") || !parent->getName().compare(0, 6, "marker")) {
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

void Scene::draw_board(double xsize, double ysize, double xoffset, double yoffset) {
	// square geometry
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	// extents of geom
	osg::ref_ptr<osg::Vec3Array> coords = new osg::Vec3Array();
	coords->push_back(osg::Vec3(0 - xoffset, 0 - yoffset, 0));
	coords->push_back(osg::Vec3(2*xsize - xoffset, 0 - yoffset, 0));
	coords->push_back(osg::Vec3(2*xsize - xoffset, 2*ysize - yoffset, 0));
	coords->push_back(osg::Vec3(0 - xoffset, 2*ysize - yoffset, 0));
	geom->setVertexArray(coords.get());
	// texture coordinates
	osg::ref_ptr<osg::Vec2Array> tcoords = new osg::Vec2Array();
	tcoords->push_back(osg::Vec2(0, 0));
	tcoords->push_back(osg::Vec2(1, 0));
	tcoords->push_back(osg::Vec2(1, 1));
	tcoords->push_back(osg::Vec2(0, 1));
	geom->setTexCoordArray(0, tcoords.get());
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	// depth
	osg::ref_ptr<osg::Depth> depth = new osg::Depth();
	depth->setFunction(osg::Depth::LEQUAL);
	depth->setRange(1.0, 1.0);
	// texture image
	osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D(osgDB::readImageFile(_path[rs::Ground]));
	tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
	tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR_MIPMAP_LINEAR);
	tex->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
	// create geode
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->addDrawable(geom.get());
	geode->getOrCreateStateSet()->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex.get(), osg::StateAttribute::ON);
	geode->setName("board");
	// add to scene
	_staging[2]->addChild(geode.get());
}

void Scene::draw_clock(double w, double h) {
	// init variables
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::Projection> projection = new osg::Projection();
	projection->setName("ClockProjection");
	osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform();
	osg::ref_ptr<osgText::Text> text = new osgText::Text();
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	double width = 0.25*w;
	double height = 0.10*h;

	// set projection matrix
	projection->setMatrix(osg::Matrix::ortho2D(0, w, 0, h));
	projection->addChild(transform.get());

	// set view matrix
	transform->setMatrix(osg::Matrix::identity());
	transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	transform->addChild(geode.get());

	// set rendering
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	geode->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");
	geode->addDrawable(text.get());
	geode->addDrawable(geom.get());

	// text
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	text->setMaximumWidth(w);
	text->setCharacterSize(20);
	text->setThreadSafeRefUnref(true);
	text->setDataVariance(osg::Object::DYNAMIC);
	text->setText(std::string("0.000000"));
	text->setAxisAlignment(osgText::Text::SCREEN);
	text->setAlignment(osgText::Text::CENTER_CENTER);
	text->setDrawMode(osgText::Text::TEXT);
	text->setPosition(osg::Vec3(w-0.5*width, h-0.5*height, 0));
	text->setColor(osg::Vec4(1, 1, 1, 1));
	text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);

	// background rectangle
	osg::Vec3Array *vertices = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(w-width, h-height, -0.1));
	vertices->push_back(osg::Vec3(w, h-height, -0.1));
	vertices->push_back(osg::Vec3(w, h, -0.1));
	vertices->push_back(osg::Vec3(w-width, h, -0.1));
	geom->setVertexArray(vertices);
	osg::Vec4Array *colors = new osg::Vec4Array();
	colors->push_back(osg::Vec4(0, 0, 0, 0.6));
	geom->setColorArray(colors, osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	geom->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// set visibility
	projection->setNodeMask((_view == Scene::FirstPerson) ? NOT_VISIBLE_MASK : VISIBLE_MASK);

	// add to scene
	_root->addChild(projection.get());
}

void Scene::draw_grid(double tics, double hash, double minx, double maxx, double miny, double maxy, double enabled) {
	if ((maxx - minx < -rs::Epsilon) || (maxy - miny < -rs::Epsilon)) return;

	// rendering bins don't seem to work.  add all to opaque_bin and let it sort
	// from top down.  first thing listed is first drawn.
	if (static_cast<int>(enabled)) {
		// grid group
		osg::ref_ptr<osg::Group> group = new osg::Group();
		group->setName("grid");

		// x- and y-axis lines
		osg::ref_ptr<osg::Geode> gridGeode3 = new osg::Geode();
		osg::ref_ptr<osg::Geometry> gridLines3 = new osg::Geometry();
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
		osg::ref_ptr<osg::Vec3Array> vertices3 = new osg::Vec3Array(4, myCoords3);
		gridLines3->setVertexArray(vertices3.get());
		gridLines3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 4));
		// set color
		osg::ref_ptr<osg::Vec4Array> colors3 = new osg::Vec4Array();
		colors3->push_back(osg::Vec4(0, 0, 0, 1));
		gridLines3->setColorArray(colors3.get());
		gridLines3->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::ref_ptr<osg::LineWidth> linewidth3 = new osg::LineWidth();
		linewidth3->setWidth(3.0f);
		// set rendering properties
		gridGeode3->getOrCreateStateSet()->setAttributeAndModes(linewidth3.get(), osg::StateAttribute::ON);
		gridGeode3->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode3->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode3->addDrawable(gridLines3.get());
		gridGeode3->setName("axes");
		group->addChild(gridGeode3.get());

		// x grid numbering
		osg::ref_ptr<osg::Billboard> xnum_billboard = new osg::Billboard();
		osg::ref_ptr<osgText::Text> xzero_text = new osgText::Text();
		char text[50];
		xzero_text->setText("0");
		xzero_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
		xzero_text->setAlignment(osgText::Text::CENTER_CENTER);
		xzero_text->setCharacterSize(30);
		xzero_text->setColor(osg::Vec4(0, 0, 0, 1));
		xzero_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
		xnum_billboard->addDrawable(xzero_text.get(), osg::Vec3d(-0.5*tics, -0.5*tics, 0.0));
		// positive
		if (maxx > rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(1.01*maxx / hash + 1); i++) {
				osg::ref_ptr<osgText::Text> xnumpos_text = new osgText::Text();
				if (_units) sprintf(text, "   %.0lf ", rs::M2CM(i*hash));
				else sprintf(text, "   %.0lf ", rs::M2IN(i*hash));
				xnumpos_text->setText(text);
				xnumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				xnumpos_text->setAlignment(osgText::Text::CENTER_TOP);
				xnumpos_text->setCharacterSize(30);
				xnumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
				xnumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				xnum_billboard->addDrawable(xnumpos_text.get(), osg::Vec3d(i*hash, 0, 0));
			}
		}
		// negative
		if (minx < -rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(fabs(1.01*minx) / hash + 1); i++) {
				osg::ref_ptr<osgText::Text> xnumneg_text = new osgText::Text();
				if (_units) sprintf(text, "%.0lf    ", rs::M2CM(-i*hash));
				else sprintf(text, "%.0lf    ", rs::M2IN(-i*hash));
				xnumneg_text->setText(text);
				xnumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				xnumneg_text->setAlignment(osgText::Text::CENTER_TOP);
				xnumneg_text->setCharacterSize(30);
				xnumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
				xnumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				xnum_billboard->addDrawable(xnumneg_text.get(), osg::Vec3d(-i*hash, 0, 0));
			}
		}
		// drawing properties
		xnum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		xnum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xnum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		// set rendering properties
		xnum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// add to scene
		xnum_billboard->setName("xnumbering");
		group->addChild(xnum_billboard.get());

		// y grid numbering
		osg::ref_ptr<osg::Billboard> ynum_billboard = new osg::Billboard();
		// positive
		if (maxy > rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(1.01*maxy / hash + 1); i++) {
				osg::ref_ptr<osgText::Text> ynumpos_text = new osgText::Text();
				if (_units) sprintf(text, "    %.0lf", rs::M2CM(i*hash));
				else sprintf(text, "    %.0lf", rs::M2IN(i*hash));
				ynumpos_text->setText(text);
				ynumpos_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				ynumpos_text->setAlignment(osgText::Text::CENTER_TOP);
				ynumpos_text->setCharacterSize(30);
				ynumpos_text->setColor(osg::Vec4(0, 0, 0, 1));
				ynumpos_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				ynum_billboard->addDrawable(ynumpos_text.get(), osg::Vec3d(0, i*hash, 0));
			}
		}
		// negative
		if (miny < -rs::Epsilon) {
			for (int i = 1; i < static_cast<int>(fabs(1.01*miny) / hash + 1); i++) {
				osg::ref_ptr<osgText::Text> ynumneg_text = new osgText::Text();
				if (_units) sprintf(text, "%.0lf    ", rs::M2CM(-i*hash));
				else sprintf(text, "%.0lf    ", rs::M2IN(-i*hash));
				ynumneg_text->setText(text);
				ynumneg_text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
				ynumneg_text->setAlignment(osgText::Text::CENTER_TOP);
				ynumneg_text->setCharacterSize(30);
				ynumneg_text->setColor(osg::Vec4(0, 0, 0, 1));
				ynumneg_text->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
				ynum_billboard->addDrawable(ynumneg_text.get(), osg::Vec3d(0, -i*hash, 0));
			}
		}
		// drawing properties
		ynum_billboard->setMode(osg::Billboard::AXIAL_ROT);
		ynum_billboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ynum_billboard->setNodeMask(~IS_PICKABLE_MASK);
		// set rendering properties
		ynum_billboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// add to scene
		ynum_billboard->setName("ynumbering");
		group->addChild(ynum_billboard.get());

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
				xbillboard->addDrawable(xtext.get(), osg::Vec3d(0.1, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext.get(), osg::Vec3d(maxx + 0.1, 0.0, 0.0));
		}
		else {
			if (minx < -rs::Epsilon)
				xbillboard->addDrawable(xtext.get(), osg::Vec3d(maxx + 0.1, 0.0, 0.0));
			else
				xbillboard->addDrawable(xtext.get(), osg::Vec3d(0.1, 0.0, 0.0));
		}
		// drawing properties
		xbillboard->setMode(osg::Billboard::AXIAL_ROT);
		xbillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		xbillboard->setNodeMask(~IS_PICKABLE_MASK);
		// set rendering properties
		xbillboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// add to scene
		xbillboard->setName("xlabel");
		group->addChild(xbillboard.get());

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
				ybillboard->addDrawable(ytext.get(), osg::Vec3d(0.0, 0.1, 0.0));
			else
				ybillboard->addDrawable(ytext.get(), osg::Vec3d(0.0, maxy + 0.1, 0.0));
		}
		else {
			if (miny < -rs::Epsilon)
				ybillboard->addDrawable(ytext.get(), osg::Vec3d(0.0, maxy + 0.1, 0.0));
			else
				ybillboard->addDrawable(ytext.get(), osg::Vec3d(0.0, 0.1, 0.0));
		}
		// drawing properties
		ybillboard->setMode(osg::Billboard::AXIAL_ROT);
		ybillboard->setAxis(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNormal(osg::Vec3d(0.0, 0.0, 1.0));
		ybillboard->setNodeMask(~IS_PICKABLE_MASK);
		// set rendering properties
		ybillboard->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// add to scene
		ybillboard->setName("ylabel");
		group->addChild(ybillboard.get());

		// grid lines for major markings
		double minx2 = static_cast<int>(ceil(((minx < -rs::Epsilon) ? 1.01 : 0.99)*minx/hash))*hash;
		double miny2 = static_cast<int>(ceil(((miny < -rs::Epsilon) ? 1.01 : 0.99)*miny/hash))*hash;
		double maxx2 = static_cast<int>(floor(((maxx < -rs::Epsilon) ? 0.99 : 1.01)*maxx/hash))*hash;
		double maxy2 = static_cast<int>(floor(((maxy < -rs::Epsilon) ? 0.99 : 1.01)*maxy/hash))*hash;
		int numx2 = static_cast<int>(ceil(1.01*(maxx2 - minx2)/hash));
		int numy2 = static_cast<int>(ceil(1.01*(maxy2 - miny2)/hash));
		int numVertices2 = 2*numx2 + 2*numy2;
		osg::ref_ptr<osg::Geode> gridGeode2 = new osg::Geode();
		osg::ref_ptr<osg::Geometry> gridLines2 = new osg::Geometry();
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
		osg::ref_ptr<osg::Vec3Array> vertices2 = new osg::Vec3Array(numVertices2, myCoords2);
		gridLines2->setVertexArray(vertices2.get());
		gridLines2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices2));
		// set color
		osg::ref_ptr<osg::Vec4Array> colors2 = new osg::Vec4Array();
		colors2->push_back(osg::Vec4(1, 0, 0, 1));
		gridLines2->setColorArray(colors2.get());
		gridLines2->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::ref_ptr<osg::LineWidth> linewidth2 = new osg::LineWidth();
		linewidth2->setWidth(2.0f);
		// set rendering properties
		gridGeode2->getOrCreateStateSet()->setAttributeAndModes(linewidth2.get(), osg::StateAttribute::ON);
		gridGeode2->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode2->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode2->addDrawable(gridLines2.get());
		gridGeode2->setName("hash");
		group->addChild(gridGeode2.get());

		// grid lines for sub-markings
		double minx1 = static_cast<int>(ceil(((minx < -rs::Epsilon) ? 1.001 : 0.999)*minx/tics))*tics;
		double miny1 = static_cast<int>(ceil(((miny < -rs::Epsilon) ? 1.001 : 0.999)*miny/tics))*tics;
		double maxx1 = static_cast<int>(floor(((maxx < -rs::Epsilon) ? 0.999 : 1.001)*maxx/tics))*tics;
		double maxy1 = static_cast<int>(floor(((maxy < -rs::Epsilon) ? 0.999 : 1.001)*maxy/tics))*tics;
		int numx1 = static_cast<int>(ceil(1.001*(maxx1 - minx1)/tics));
		int numy1 = static_cast<int>(ceil(1.001*(maxy1 - miny1)/tics));
		int numVertices = 2*numx1 + 2*numy1;
		osg::ref_ptr<osg::Geode> gridGeode = new osg::Geode();
		osg::ref_ptr<osg::Geometry> gridLines = new osg::Geometry();
		osg::Vec3 *myCoords = new osg::Vec3[numVertices]();
		// draw x lines
		for (int i = 0, j = 0; i < numx1; i++) {
			myCoords[j++] = osg::Vec3(minx1 + i*tics, miny, 0);
			myCoords[j++] = osg::Vec3(minx1 + i*tics, maxy, 0);
		}
		// draw y lines
		for (int i = 0, j = 2*numx1; i < numy1; i++) {
			myCoords[j++] = osg::Vec3(minx, miny1 + i*tics, 0);
			myCoords[j++] = osg::Vec3(maxx, miny1 + i*tics, 0);
		}
		// add vertices
		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(numVertices, myCoords);
		gridLines->setVertexArray(vertices.get());
		gridLines->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numVertices));
		// set color
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
		colors->push_back(osg::Vec4(1, 1, 1, 1));
		gridLines->setColorArray(colors.get());
		gridLines->setColorBinding(osg::Geometry::BIND_OVERALL);
		// set line width
		osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
		linewidth->setWidth(1);
		// set rendering properties
		gridGeode->getOrCreateStateSet()->setAttributeAndModes(linewidth.get(), osg::StateAttribute::ON);
		gridGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
		// enable shadowing
		//gridGeode->setNodeMask( (RECEIVES_SHADOW_MASK & ~IS_PICKABLE_MASK) );
		// add to scene
		gridGeode->addDrawable(gridLines.get());
		gridGeode->setName("tics");
		group->addChild(gridGeode.get());

		// add group to staging
		_staging[2]->addChild(group.get());

		// clean up
		delete myCoords2;
		delete myCoords;
	}
}

void Scene::draw_ground(void) {
	// depth
	osg::ref_ptr<osg::Depth> depth = new osg::Depth();
	depth->setFunction(osg::Depth::LEQUAL);
	depth->setRange(1.0, 1.0);

	// geode
	osg::ref_ptr<osg::Node> geode = osgDB::readNodeFile(_path[rs::Ground]);

	// rendering properties
	osg::ref_ptr<osg::StateSet> ss = new osg::StateSet();
	ss->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
	ss->setRenderBinDetails(0, "RenderBin");
	ss->setAttribute(create_material(osg::Vec4(0.298, 0.424, 0.200, 1)), osg::StateAttribute::OVERRIDE);
	geode->setStateSet(ss.get());

	// transform
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setScale(osg::Vec3d(10, 10, 0));
	transform->addChild(geode.get());
	transform->setName("ground");

	// add to scene
	_staging[2]->addChild(transform.get());
}

void Scene::draw_hud(double w, double h, bool paused) {
	// init variables
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	osg::ref_ptr<osg::Projection> projection = new osg::Projection();
	projection->setName("HUDProjection");
	osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform();
	osg::ref_ptr<osgText::Text> text = new osgText::Text();
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	double p = 0.1;

	// set projection matrix
	projection->setMatrix(osg::Matrix::ortho2D(0, w, 0, h));
	projection->addChild(transform.get());

	// set view matrix
	transform->setMatrix(osg::Matrix::identity());
	transform->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	transform->addChild(geode.get());

	// set rendering
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	geode->getOrCreateStateSet()->setRenderBinDetails(100, "RenderBin");
	geode->addDrawable(text.get());
	geode->addDrawable(geom.get());

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
	osg::Vec3Array *vertices = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(0, 0, -0.1));
	vertices->push_back(osg::Vec3(w, 0, -0.1));
	vertices->push_back(osg::Vec3(w, p*h, -0.1));
	vertices->push_back(osg::Vec3(0, p*h, -0.1));
	geom->setVertexArray(vertices);
	osg::Vec4Array *colors = new osg::Vec4Array();
	colors->push_back(osg::Vec4(0, 0, 0, 0.6));
	geom->setColorArray(colors, osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	geom->getOrCreateStateSet()->setMode(GL_BLEND,osg::StateAttribute::ON);
	geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// set visibility
	projection->setNodeMask((_view == Scene::FirstPerson) ? NOT_VISIBLE_MASK : VISIBLE_MASK);

	// add to scene
	_root->addChild(projection.get());
}

void Scene::draw_skybox(void) {
	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	osg::ref_ptr<osg::TexEnv> te = new osg::TexEnv();
	te->setMode(osg::TexEnv::REPLACE);
	stateset->setTextureAttributeAndModes(0, te.get(), osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexGen> tg = new osg::TexGen();
	tg->setMode(osg::TexGen::NORMAL_MAP);
	stateset->setTextureAttributeAndModes(0, tg.get(), osg::StateAttribute::ON);
	osg::ref_ptr<osg::TexMat> tm = new osg::TexMat();
	stateset->setTextureAttribute(0, tm.get());
	osg::ref_ptr<osg::TextureCubeMap> skymap = new osg::TextureCubeMap();
	osg::Image *imagePosX = osgDB::readImageFile(_path[rs::RightSide]);
	osg::Image *imageNegX = osgDB::readImageFile(_path[rs::LeftSide]);
	osg::Image *imagePosY = osgDB::readImageFile(_path[rs::Top]);
	osg::Image *imageNegY = osgDB::readImageFile(_path[rs::Top]);
	osg::Image *imagePosZ = osgDB::readImageFile(_path[rs::Front]);
	osg::Image *imageNegZ = osgDB::readImageFile(_path[rs::Back]);
	if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ) {
		skymap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
		skymap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
		skymap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
		skymap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);
		skymap->setUnRefImageDataAfterApply(true);
		skymap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
		skymap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
	}
	stateset->setTextureAttributeAndModes(0, skymap.get(), osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth();
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setRange(1.0, 1.0);
	stateset->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
	stateset->setRenderBinDetails(-1, "RenderBin");
	stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::ref_ptr<osg::Drawable> drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 1));
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();
	geode->setCullingActive(false);
	geode->setStateSet(stateset.get());
	geode->addDrawable(drawable.get());
	geode->setCullCallback(new TextureCallback(*tm));
	osg::ref_ptr<osg::Transform> transform = new SkyTransform();
	transform->setCullingActive(false);
	transform->addChild(geode.get());
	transform->setName("skybox");
	_staging[2]->addChild(transform.get());
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
	p->_thread = true;

	// viewer event handlers
	p->_viewer->addEventHandler(new osgViewer::WindowSizeHandler());

	// signal calling function that setup is done
	RS_COND_ACTION(&(p->_graphics_cond), &(p->_graphics_mutex), p->_graphics = true);

	// run viewer
	RS_MUTEX_LOCK(&(p->_thread_mutex));
	while (p->_thread && !p->_viewer->done()) {
		RS_MUTEX_UNLOCK(&(p->_thread_mutex));

		// start clock for frame rate calculation
		double minFrameTime = (p->_rate > 0.0) ? 1.0 / p->_rate : 0.0;
		osg::Timer_t startFrameTick = osg::Timer::instance()->tick();

		// update thread & scene elements
		p->_viewer->frame();
		RS_MUTEX_LOCK(&(p->_thread_mutex));
		p->addAndRemoveChildren();
		RS_MUTEX_UNLOCK(&(p->_thread_mutex));

		if (p->_view == Scene::FirstPerson) {
			// update camera
			osg::Group *test = NULL;
			for (unsigned int i = 0; i < p->_scene->getNumChildren(); i++) {
				test = dynamic_cast<osg::Group *>(p->_scene->getChild(i));
				if (test && (!test->getName().compare(0, 6, "robot0"))) {
					osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(test->getChild(2 + i));
					rsScene::FixedManipulator *manip = dynamic_cast<rsScene::FixedManipulator*>(p->_viewer->getCameraManipulator());
					manip->setRotation(osg::Quat(0, sin(rs::Pi/4), 0, cos(rs::Pi/4))*pat->getAttitude()*osg::Quat(0, 0, sin(rs::Pi/2), cos(rs::Pi/2)));
					break;
				}
			}

			// process next frame
			p->_sch->captureNextFrame(*(p->_viewer));
		}
		else {
			// update clock
			p->updateClock();
		}

		// pause for proper frame rate
		osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
		double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
		if (frameTime < minFrameTime) OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0*(minFrameTime - frameTime)));

		// unlock
		RS_MUTEX_LOCK(&(p->_thread_mutex));
	}
	RS_MUTEX_UNLOCK(&(p->_thread_mutex));

	// clean up viewer & root
	p->_viewer->setSceneData(NULL);

	// return
	return arg;
}

bool Scene::intersect_new_item(std::string name, const osg::BoundingBox &bb) {
	// find nodes of intersection
	osg::Group *test = NULL;
	bool retval = false;
	for (unsigned int i = 0; i < _scene->getNumChildren(); i++) {
		test = dynamic_cast<osg::Group *>(_scene->getChild(i));
		// get preconfig node
		if (test && (!test->getName().compare(0, 3, "pre")) && test->getName() != name) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			if (bb.intersects(cbbv.getBoundingBox())) {
				this->toggleHighlight(test, test, rs::Vec(1, 0, 0), true);
				retval = true;
			}
		}
		// get robot node
		else if (test && (!test->getName().compare(0, 5, "robot")) && (test->getName() != name)) {
			osg::ComputeBoundsVisitor cbbv;
			test->accept(cbbv);
			osg::BoundingBox bb2 = cbbv.getBoundingBox();
			if (	std::max(bb.xMin(), bb2.xMin()) + 0.001 <= std::min(bb.xMax(), bb2.xMax()) - 0.001 &&
					std::max(bb.yMin(), bb2.yMin()) + 0.001 <= std::min(bb.yMax(), bb2.yMax()) - 0.001 &&
					std::max(bb.zMin(), bb2.zMin()) + 0.001 <= std::min(bb.zMax(), bb2.zMax()) - 0.001) {
				this->toggleHighlight(test, test->getChild(2)->asTransform()->getChild(0), rs::Vec(1, 0, 0), true);
				retval = true;
			}
		}
		// get obstacle node
		else if (test && !test->getName().compare(0, 8, "obstacle") && (test->getName() != name)) {
			if (test->getNumChildren() > 1) {
				for (unsigned int i = 0; i < test->getNumChildren(); i++) {
					osg::Group *test2 = test->getChild(i)->asGroup();
					osg::ComputeBoundsVisitor cbbv;
					test2->accept(cbbv);
					osg::BoundingBox bb2 = cbbv.getBoundingBox();
					if (	std::max(bb.xMin(), bb2.xMin()) + 0.001 <= std::min(bb.xMax(), bb2.xMax()) - 0.001 &&
							std::max(bb.yMin(), bb2.yMin()) + 0.001 <= std::min(bb.yMax(), bb2.yMax()) - 0.001 &&
							std::max(bb.zMin(), bb2.zMin()) + 0.001 <= std::min(bb.zMax(), bb2.zMax()) - 0.001) {
						this->toggleHighlight(test, test->getChild(i), rs::Vec(1, 0, 0), true);
						retval = true;
						break;
					}
				}
			}
			else {
				osg::ComputeBoundsVisitor cbbv;
				test->accept(cbbv);
				if (bb.intersects(cbbv.getBoundingBox())) {
					this->toggleHighlight(test, test->getChild(0), rs::Vec(1, 0, 0), true);
					retval = true;
				}
			}
		}
	}
	return retval;
}

