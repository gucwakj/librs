#include <osg/Group>
#include <osg/PositionAttitudeTransform>

#include <rsCallback/GroundCallback>

using namespace rsCallback;

GroundCallback::GroundCallback(rsSim::Obstacle *ground) {
	_ground = ground;
}

void GroundCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos = dBodyGetPosition(*_ground);
		const double *quat = dBodyGetQuaternion(*_ground);
		osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	}
	traverse(node, nv);
}

