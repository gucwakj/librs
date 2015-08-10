#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osgText/Text>

#include <rsScene/Scene>
#include <rsCallback/Dof>

using namespace rsCallback;
using namespace rsDof;

Dof::Dof(rsSim::Dof *robot, rsSim::BodyList &bodies, rsSim::ConnectorList &conn, bool units) {
	_bodies = bodies;
	_conn = conn;
	_count = 1;
	_robot = robot;
	_units = units;
}

void Dof::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group && _robot && &_bodies && &_conn) {
		// child 0: hud
		std::string text;
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		if (_robot->getName().size())
			text.append(_robot->getName());
		else
			text.append("Robot " + std::to_string(_robot->getID()+1));
		if (_units)
			text.append("\n\n(" + std::to_string(x*100) + ", " + std::to_string(y*100) + ") [cm]");
		else
			text.append("\n\n(" + std::to_string(y*39.37) + ", " + std::to_string(y*39.37) + ") [in]");
		osgText::Text *label = dynamic_cast<osgText::Text *>(group->getChild(0)->asGeode()->getDrawable(0));
		label->setText(text);
		label->setPosition(osg::Vec3(x, y, z));
		// child 1: tracking line
		if (_robot->getTrace()) {
			osg::Geode *geode2 = dynamic_cast<osg::Geode *>(group->getChild(1));
			geode2->setNodeMask(VISIBLE_MASK);
			osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode2->getDrawable(0)->asGeometry());
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			double *c = _robot->getRGB();
			colors->push_back(osg::Vec4(c[0], c[1], c[2], 1));
			osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
			vertices->push_back(osg::Vec3(x, y, 0));
			osg::DrawArrays *array = dynamic_cast<osg::DrawArrays *>(draw->getPrimitiveSet(0));
			array->setCount(_count++);
		}
		// child 2->2+NUM_PARTS: bodies
		const double *pos, *quat;
		osg::PositionAttitudeTransform *pat;
		for (int i = 0; i < Bodies::Num_Parts; i++) {
			pos = dBodyGetPosition(_bodies[i]);
			quat = dBodyGetQuaternion(_bodies[i]);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + i));
			if (pat) {
				pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
				pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
			}
		}
		// child 2: bodies; drawable 2: led
		osg::ShapeDrawable *led = dynamic_cast<osg::ShapeDrawable *>(group->getChild(2)->asTransform()->getChild(1)->asGeode()->getDrawable(0));
		double *rgb = _robot->getRGB();
		led->setColor(osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0));
		// child 7->end: connectors
		for (unsigned int i = 0; i < _conn.size(); i++) {
			if (_conn[i].body) {
				pos = dBodyGetPosition(_conn[i].body);
				quat = dBodyGetQuaternion(_conn[i].body);
				pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + Bodies::Num_Parts + i));
				pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
				pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
			}
		}
	}
	traverse(node, nv);
}

