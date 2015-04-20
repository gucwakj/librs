#include <osg/Group>
#include <osg/PositionAttitudeTransform>

#include <rsCallback/ObstacleCallback>

using namespace rsCallback;

ObstacleCallback::ObstacleCallback(rsSim::Obstacle *obstacle) {
	_obstacle = obstacle;
}

void ObstacleCallback::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		const double *pos = dBodyGetPosition(*_obstacle);
		const double *quat = dBodyGetQuaternion(*_obstacle);
		osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
		pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
		pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
	}
	traverse(node, nv);
}

