#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osgText/Text>

#include <rsCallback/Mindstorms>
#include <rsScene/Scene>

using namespace rsCallback;
using namespace rsMindstorms;

/**********************************************************
	public functions
 **********************************************************/
void Mindstorms::setCallbackParams(rsSim::Robot *robot, rsSim::BodyList &bodies, bool units) {
	_bodies = bodies;
	_robot = dynamic_cast<rsSim::Mindstorms *>(robot);
	_units = units;
}

void Mindstorms::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group && _robot && &_bodies) {
		// child 0: hud
		std::string text;
		// get position
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		// set name or robot+id
		if (_robot->getName().size())
			text.append(_robot->getName());
		else
			text.append("Robot " + std::to_string(_robot->getID()+1));
		// position
		if (_units)
			text.append("\n\n(" + std::to_string(rs::M2CM(x)) + ", " + std::to_string(rs::M2CM(y)) + ") [cm]");
		else
			text.append("\n\n(" + std::to_string(rs::M2IN(y)) + ", " + std::to_string(rs::M2IN(y)) + ") [in]");
		// draw label
		osgText::Text *label = dynamic_cast<osgText::Text *>(group->getChild(0)->asGeode()->getDrawable(0));
		label->setText(text);
		label->setPosition(osg::Vec3(x, y, z));
		// child 1: trace
		if (_robot->getTrace()) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			float *c = _robot->getRGB();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], 1));
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// child 2->2+NUM_PARTS: bodies
		for (int i = 0; i < Bodies::Num_Parts; i++) {
			const double *pos = dBodyGetPosition(_bodies[i]);
			const double *quat = dBodyGetQuaternion(_bodies[i]);
			osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// child 2: bodies; drawable 2: led
		osg::ShapeDrawable *led = dynamic_cast<osg::ShapeDrawable *>(group->getChild(2)->asTransform()->getChild(1)->asGeode()->getDrawable(0));
		float *rgb = _robot->getRGB();
		led->setColor(osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0));
	}
	traverse(node, nv);
}

