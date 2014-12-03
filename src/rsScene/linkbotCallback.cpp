#include "linkbotCallback.hpp"
#include "linkbot.hpp"

using namespace rsScene;

linkbotCallback::linkbotCallback(CLinkbotT *robot, osg::ShapeDrawable *led) {
	_robot = robot;
	_led = led;
	_count = 1;
}

void linkbotCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos, *quat;
		int i, k = 0;
		osg::PositionAttitudeTransform *pat;
		// draw body parts
		for (i = 2; i < 2+4; i++) {
			pos = dBodyGetPosition(_robot->getBodyID(i-2));
			quat = dBodyGetQuaternion(_robot->getBodyID(i-2));
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
		// draw connectors
		for (int j = 0; j < _robot->_conn.size(); j++) {
			dMatrix3 R;
			dQuaternion Q;
			double p[3] = {0};
			if (_robot->_conn[j]->d_side != -1) {
				_robot->getFaceParams(_robot->_conn[j]->face, R, p);
				_robot->getConnectorParams(_robot->_conn[j]->d_type, _robot->_conn[j]->d_side, R, p);
				p[0] += R[0]*_robot->_conn[j]->o[0] + R[1]*_robot->_conn[j]->o[1] + R[2]*_robot->_conn[j]->o[2];
				p[1] += R[4]*_robot->_conn[j]->o[0] + R[5]*_robot->_conn[j]->o[1] + R[6]*_robot->_conn[j]->o[2];
				p[2] += R[8]*_robot->_conn[j]->o[0] + R[9]*_robot->_conn[j]->o[1] + R[10]*_robot->_conn[j]->o[2];
				dRtoQ(R, Q);
			}
			else {
				const double *pos = dBodyGetPosition(_robot->_conn[j]->body);
				p[0] = pos[0]; p[1] = pos[1]; p[2] = pos[2];
				const double *rot = dBodyGetQuaternion(_robot->_conn[j]->body);
				Q[0] = rot[0]; Q[1] = rot[1]; Q[2] = rot[2]; Q[3] = rot[3];
			}
			pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i + k++));
			pat->setPosition(osg::Vec3d(p[0], p[1], p[2]));
			pat->setAttitude(osg::Quat(Q[1], Q[2], Q[3], Q[0]));
		}
		// draw hud
		osg::Geode *geode = dynamic_cast<osg::Geode *>(group->getChild(0));
		osgText::Text *label = dynamic_cast<osgText::Text *>(geode->getDrawable(0));
		char text[50];
	//	if (g_sim->getUnits()) {
	//		sprintf(text, "Robot %d\n(%.4lf, %.4lf) [in]", _robot->getID()+1,
	//			_robot->getCenter(0)*39.37, _robot->getCenter(1)*39.37);
	//	}
	//	else {
			sprintf(text, "Robot %d\n(%.4lf, %.4lf) [cm]", _robot->getID()+1,
				_robot->getCenter(0)*100, _robot->getCenter(1)*100);
		//}
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
			osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(draw->getColorArray());
			colors->pop_back();
			colors->push_back(osg::Vec4(_robot->_rgb[0], _robot->_rgb[1], _robot->_rgb[2], 1.0f) );
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

