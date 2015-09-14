#include <osg/Geode>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <rs/Macros>
#include <rsScene/Linkbot>

using namespace rsScene;
using namespace rsLinkbot;

Linkbot::Linkbot(int form) : rsRobots::Robot(form), rsRobots::Linkbot(form) { }

Linkbot::~Linkbot(void) { }

/**********************************************************
	public functions
 **********************************************************/
void Linkbot::draw(Group *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// initialize variables
	osg::ref_ptr<osg::Node> body[rsLinkbot::Bodies::Num_Parts];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[rsLinkbot::Bodies::Num_Parts];

	// create transforms
	for (int i = 0; i < rsLinkbot::Bodies::Num_Parts; i++) {
		pat[i] = new osg::PositionAttitudeTransform();
		group->addChild(pat[i]);
	}

	// draw body
	body[rsLinkbot::Bodies::Body] = osgDB::readNodeFile(_model_path + "linkbot/body.3ds");
	body[rsLinkbot::Bodies::Body]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0.867, 0.827, 0.776, 1)));
	pat[rsLinkbot::Bodies::Body]->setPosition(osg::Vec3d(p[0], p[1], p[2]));
	pat[rsLinkbot::Bodies::Body]->setAttitude(osg::Quat(q[0], q[1], q[2], q[3]));

	// draw 'led'
	osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder(osg::Vec3d(0, -0.02, 0.0308), 0.01, 0.01);
	osg::ref_ptr<osg::ShapeDrawable> led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(c[0], c[1], c[2], 1));
	osg::ref_ptr<osg::Geode> bodyled = new osg::Geode();
	bodyled->addDrawable(led);
	bodyled->getOrCreateStateSet()->setRenderBinDetails(33, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	bodyled->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	bodyled->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	bodyled->getOrCreateStateSet()->setMode(GL_ALPHA_TEST, osg::StateAttribute::ON);
	bodyled->setCullingActive(false);

	// draw cap 1
	rs::Quat q1 = this->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap1, rs::D2R(a[rsLinkbot::Bodies::Joint1]), q);
	rs::Pos p1 = this->getRobotBodyPosition(rsLinkbot::Bodies::Cap1, p, q);
	body[rsLinkbot::Bodies::Cap1] = osgDB::readNodeFile(_model_path + "linkbot/face_rotate.3ds");
	body[rsLinkbot::Bodies::Cap1]->getOrCreateStateSet()->setAttribute(create_material(osg::Vec4(0, 0, 0, 1)));
	pat[rsLinkbot::Bodies::Cap1]->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	pat[rsLinkbot::Bodies::Cap1]->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// draw cap 2
	q1 = this->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap2, rs::D2R(a[rsLinkbot::Bodies::Joint2]), q);
	p1 = this->getRobotBodyPosition(rsLinkbot::Bodies::Cap2, p, q);
	if (this->getForm() == rs::LinkbotI) {
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
	q1 = this->getRobotBodyQuaternion(rsLinkbot::Bodies::Cap3, rs::D2R(a[rsLinkbot::Bodies::Joint3]), q);
	p1 = this->getRobotBodyPosition(rsLinkbot::Bodies::Cap3, p, q);
	if (this->getForm() == rs::LinkbotL) {
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

	// set masks
	//this->setNodeMask(CASTS_SHADOW_MASK);
	//this->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	group->insertChild(0, this->create_hud(p));

	// tracking node
	group->insertChild(1, this->create_tracking_line(p, c, trace));

	// set robot name
	group->setName(std::string("robot").append(std::to_string(this->getID())));
}

void Linkbot::drawConnector(Group *group, int type, int face, int orientation, double size, int side, int conn) {
	// get robot p&q
	osg::PositionAttitudeTransform *pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsLinkbot::Bodies::Body));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();

	// get face p&q
	rs::Quat Q1 = this->getRobotBodyQuaternion(face, 0, rs::Quat(q[0], q[1], q[2], q[3]));
	rs::Pos P1 = this->getRobotFacePosition(face, rs::Pos(p[0], p[1], p[2]), rs::Quat(q[0], q[1], q[2], q[3]));
	if (conn == -1) {
		P1 = this->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
	}
	else {
		P1 = this->getConnFacePosition(type, side, orientation, P1, Q1);
		Q1 = this->getConnFaceQuaternion(type, side, orientation, Q1);
		type = conn;
		P1 = this->getConnBodyPosition(type, orientation, P1, Q1);
		Q1 = this->getConnBodyQuaternion(type, orientation, Q1);
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
				transform->setScale(osg::Vec3d(1, 1, this->getCasterScale()));
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
			transform->setScale(osg::Vec3d(1, this->getWheelRatio(rsLinkbot::Connectors::TinyWheel), this->getWheelRatio(rsLinkbot::Connectors::TinyWheel)));
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

