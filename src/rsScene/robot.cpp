#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>
#include <osg/Point>
#include <osgText/Text>

#include <rs/Macros>
#include <rsScene/Robot>

using namespace rsScene;

Robot::Robot(void) : rsRobots::Robot(rs::Robot) {
	_units = false;			// customary
}

Robot::~Robot(void) { }

/**********************************************************
	public functions
 **********************************************************/
void Robot::setModelPath(std::string path) {
	_model_path = path;
}

void Robot::setUnits(bool units) {
	_units = units;
}

/**********************************************************
	protected functions
 **********************************************************/
osg::Geode* Robot::create_hud(const rs::Pos &p) {
	// create text for HUD
	std::string text;
	if (this->getName().size())
		text.append(this->getName());
	else
		text.append("Robot " + std::to_string(this->getID()+1));
	if (_units)
		text.append("\n\n(" + std::to_string(rs::M2CM(p[0])) + ", " + std::to_string(rs::M2CM(p[1])) + ") [cm]");
	else
		text.append("\n\n(" + std::to_string(rs::M2IN(p[0])) + ", " + std::to_string(rs::M2IN(p[1])) + ") [in]");

	// osg label for the text
	osg::ref_ptr<osgText::Text> label = new osgText::Text();
	label->setText(text);
	label->setAlignment(osgText::Text::CENTER_CENTER);
	label->setAxisAlignment(osgText::Text::SCREEN);
	label->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_CENTER);
	label->setBoundingBoxColor(osg::Vec4(0, 0, 0, 0.9));
	label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	label->setCharacterSize(30);
	label->setColor(osg::Vec4(1, 1, 1, 1));
	label->setDrawMode(osgText::Text::TEXT | osgText::Text::FILLEDBOUNDINGBOX);
	label->setPosition(osg::Vec3(p[0], p[1], p[2] + (this->getID() % 2 ? 0.08 : 0) + 0.08));

	// set rendering properties
	osg::Geode *geode = new osg::Geode();
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderBinDetails(22, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	// set geode properties
	geode->addDrawable(label.get());
	geode->setNodeMask(NOT_VISIBLE_MASK);
	geode->setName("robotHUD");

	// return
	return geode;
}

osg::Material* Robot::create_material(osg::Vec4 color) {
	osg::Material *material = new osg::Material();
	material->setAmbient(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setEmission(osg::Material::FRONT, color);
	material->setShininess(osg::Material::FRONT, 2);
	material->setSpecular(osg::Material::FRONT, osg::Vec4(1, 1, 1, 1));
	return material;
}

osg::Geode* Robot::create_trace(const rs::Pos &p, const rs::Vec &c, bool trace) {
	// create line geometry
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	geom->setDataVariance(osg::Object::DYNAMIC);
	geom->setUseDisplayList(false);
	geom->insertPrimitiveSet(0, new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, 1, 1));

	// set vertices
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
	vertices->push_back(osg::Vec3(p[0], p[1], 0));
	geom->setVertexArray(vertices.get());

	// set color
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
	colors->push_back(osg::Vec4(c[0], c[1], c[2], c[3]));
	geom->setColorArray(colors.get());
	geom->setColorBinding(osg::Geometry::BIND_OVERALL);

	// set rendering properties
	osg::Geode *geode = new osg::Geode();
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(3), osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setRenderBinDetails(20, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);

	// set geode properties
	geode->addDrawable(geom.get());
	geode->setNodeMask(((trace) ? VISIBLE_MASK : NOT_VISIBLE_MASK));
	geode->setName("robotTrace");

	// return
	return geode;
}

