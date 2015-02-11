#include <rsCallback/linkbotCallback.hpp>

using namespace rsCallback;

linkbotCallback::linkbotCallback(rsSim::LinkbotT *robot, rsSim::BodyList &bodies, rsSim::ConnectorList &conn, bool units) {
	_bodies = bodies;
	_conn = conn;
	_count = 1;
	_robot = robot;
	_units = units;
}

void linkbotCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		// child 0: hud
		osgText::Text *label = dynamic_cast<osgText::Text *>(group->getChild(0)->asGeode()->getDrawable(0));
		char text[50];
		if (_units)
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1, _robot->getCenter(0)*100, _robot->getCenter(1)*100);
		else
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1, _robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
		label->setText(text);
		double x = _robot->getCenter(0);
		double y = _robot->getCenter(1);
		double z = _robot->getCenter(2) + (_robot->getID() % 2 ? 0.08 : 0) + 0.08;
		label->setPosition(osg::Vec3(x, y, z));
		// child 1: tracking line
		/*if (_robot->_trace) {
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
		}*/
		// child 2->2+NUM_PARTS: bodies
		const double *pos, *quat;
		osg::PositionAttitudeTransform *pat;
		for (int i = 0; i < rsRobots::LinkbotT::NUM_PARTS; i++) {
			pos = dBodyGetPosition(_bodies[i]);
			quat = dBodyGetQuaternion(_bodies[i]);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// child 2: bodies; drawable 2: led
		//osg::ShapeDrawable *led = dynamic_cast<osg::ShapeDrawable *>(group->getChild(2)->asTransform()->getChild(0)->asGeode()->getDrawable(2));
		//double *rgb = _robot->getRGB();
		//led->setColor(osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0));
		// child 7->end: connectors
		for (int i = 0; i < _conn.size(); i++) {
			pos = dBodyGetPosition(_conn[i]->body);
			quat = dBodyGetQuaternion(_conn[i]->body);
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(2 + rsRobots::LinkbotT::NUM_PARTS + i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
	}
	traverse(node, nv);
}

