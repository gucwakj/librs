#include "mobotCallback.hpp"
#include "mobot.hpp"

using namespace rsCallback;

mobotCallback::mobotCallback(CMobot *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void mobotCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
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
		if (g_sim->getUnits()) {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
				_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		}
		else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		}
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

