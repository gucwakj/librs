#include <osg/Geode>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <rs/Macros>
#include <rsScene/Mindstorms>

using namespace rsScene;
using namespace rsMindstorms;

Mindstorms::Mindstorms(int form) : rsRobots::Robot(form), rsRobots::Mindstorms(form) { }

Mindstorms::~Mindstorms(void) { }

/**********************************************************
	public functions
 **********************************************************/
void Mindstorms::draw(Group *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// create transform
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();
	group->addChild(pat.get());

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
	osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder(osg::Vec3d(0, 0.02, 0), 0.01, 0.01);
	osg::ref_ptr<osg::ShapeDrawable> led = new osg::ShapeDrawable(cyl);
	led->setColor(osg::Vec4(c[0], c[1], c[2], 1));
	osg::ref_ptr<osg::Geode> bodyled = new osg::Geode();
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

void Mindstorms::drawWheel(Group *group, int type, int face) {
	// get robot p&q
	osg::ref_ptr<osg::PositionAttitudeTransform> pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsMindstorms::Bodies::Body));
	osg::Vec3d p = pat->getPosition();
	osg::Quat q = pat->getAttitude();

	// wheel position
	rs::Quat q1 = this->getRobotBodyQuaternion(face, 0, rs::Quat(q[0], q[1], q[2], q[3]));
	rs::Pos p1 = this->getRobotBodyPosition(face, rs::Pos(p[0], p[1], p[2]), rs::Quat(q[0], q[1], q[2], q[3]));

	// PAT to transform mesh
	osg::ref_ptr<osg::PositionAttitudeTransform> transform = new osg::PositionAttitudeTransform();
	transform->setPosition(osg::Vec3d(p1[0], p1[1], p1[2]));
	transform->setAttitude(osg::Quat(q1[0], q1[1], q1[2], q1[3]));

	// wheel
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(_model_path + "mindstorms/wheel.3ds");
	transform->setScale(osg::Vec3d(1, this->getWheelRatio(type), this->getWheelRatio(type)));
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

