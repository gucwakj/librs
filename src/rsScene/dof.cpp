#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgText/Text>

#include <rs/Macros>
#include <rsScene/Dof>

using namespace rsScene;
using namespace rsDof;

Dof::Dof(int form) : rsRobots::Robot(form), rsRobots::Dof(form), rsScene::Robot() { }

Dof::~Dof(void) { }

/**********************************************************
	public functions
 **********************************************************/
void Dof::draw(Group *group, const rs::Pos &p, const rs::Quat &q, const rs::Vec &a, const rs::Vec &c, bool trace) {
	// initialize variables
	osg::ref_ptr<osg::Node> body[rsDof::Bodies::Num_Parts];
	osg::ref_ptr<osg::PositionAttitudeTransform> pat[rsDof::Bodies::Num_Parts];
	int enabled = dynamic_cast<rsRobots::Dof *>(this)->getEnabled();

	// create transforms
	for (int i = 0; i < rsDof::Bodies::Num_Parts; i++) {
		pat[i] = new osg::PositionAttitudeTransform;
		group->addChild(pat[i]);
	}

	// set tracing
	//robot->setTrace(trace);

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
	rs::Quat q1 = this->getRobotBodyQuaternion(enabled, rs::D2R(a[0]), q);
	rs::Pos p1 = this->getRobotBodyPosition(enabled, p, q);
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

	// set masks
	//robot->setNodeMask(CASTS_SHADOW_MASK);
	//robot->setNodeMask(IS_PICKABLE_MASK);

	// draw HUD
	osgText::Text *label = new osgText::Text();
	std::string text;
	if (this->getName().size())
		text.append(this->getName());
	else
		text.append("Robot " + std::to_string(this->getID()+1));
	if (_units)
		text.append("\n\n(" + std::to_string(p[0]*100) + ", " + std::to_string(p[1]*100) + ") [cm]");
	else
		text.append("\n\n(" + std::to_string(p[0]*39.37) + ", " + std::to_string(p[1]*39.37) + ") [in]");
	label->setText(text);
	label->setPosition(osg::Vec3(p[0], p[1], p[2] + (this->getID() % 2 ? 0.08 : 0) + 0.08));
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
	group->setName(std::string("robot").append(std::to_string(this->getID())));
}

void Dof::drawConnector(Group *group, int type, int face, int orientation, double size, int side, int conn) {
	// get robot p&q
	osg::PositionAttitudeTransform *pat;
	pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsDof::Bodies::Body));
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

