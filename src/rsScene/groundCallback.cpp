#include "groundCallback.hpp"

using namespace rsScene;

groundCallback::groundCallback(Ground *ground) {
	_ground = ground;
}

void groundCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos = dBodyGetPosition(_ground->body);
		const double *quat = dBodyGetQuaternion(_ground->body);
		osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	}
	traverse(node, nv);
}

