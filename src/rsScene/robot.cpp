#include <rs/Macros>
#include <rsScene/Scene>
#include <rsScene/Robot>

using namespace rsScene;

Robot::Robot(void) : rsRobots::Robot(rs::ROBOT) {
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
osg::Material* Robot::create_material(osg::Vec4 color) {
	osg::Material *material = new osg::Material();
	material->setAmbient(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setDiffuse(osg::Material::FRONT, osg::Vec4(0, 0, 0, 1));
	material->setEmission(osg::Material::FRONT, color);
	material->setShininess(osg::Material::FRONT, 2);
	material->setSpecular(osg::Material::FRONT, osg::Vec4(1, 1, 1, 1));
	return material;
}

