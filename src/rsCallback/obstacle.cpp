#include <osg/Group>
#include <osg/PositionAttitudeTransform>

#include <rsCallback/Obstacle>

using namespace rsCallback;

Obstacle::Obstacle(rsSim::Obstacle *obstacle) {
	_obstacle = obstacle;
}

/**********************************************************
	public functions
 **********************************************************/
void Obstacle::operator()(osg::Node *node, osg::NodeVisitor *nv) {
	osg::Group *group = dynamic_cast<osg::Group *>(node);
	if (group) {
		if (group->getNumChildren() > 1) {
			for (unsigned int i = 0; i < group->getNumChildren(); i++) {
				const double *pos;
				dQuaternion quat;
				dGeomID geom1, geom2;
				if (i == 0) {
					geom1 = dBodyGetFirstGeom(*_obstacle);
					pos = dGeomGetPosition(geom1);
					dGeomGetQuaternion(geom1, quat);
				}
				else {
					geom2 = dBodyGetNextGeom(geom1);
					pos = dGeomGetPosition(geom2);
					dGeomGetQuaternion(geom2, quat);
					geom1 = geom2;
				}
				osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(i)->asGroup()->getChild(0));
				pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
				pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
			}
		}
		else {
			const double *pos = dBodyGetPosition(*_obstacle);
			const double *quat = dBodyGetQuaternion(*_obstacle);
			osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *>(group->getChild(0));
			pat->setPosition(osg::Vec3d(pos[0], pos[1], pos[2]));
			pat->setAttitude(osg::Quat(quat[1], quat[2], quat[3], quat[0]));
		}
	}
	traverse(node, nv);
}

