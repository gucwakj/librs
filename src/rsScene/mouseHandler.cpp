#include "mouseHandler.hpp"

using namespace rsScene;

mouseHandler::mouseHandler(void) {
	_mx = 0.0;
	_my = 0.0;
}

bool mouseHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	if (!viewer) return false;

	switch (ea.getEventType()) {
		case(osgGA::GUIEventAdapter::PUSH):
		case(osgGA::GUIEventAdapter::MOVE):
			_mx = ea.getX();
			_my = ea.getY();
			return false;
		case(osgGA::GUIEventAdapter::RELEASE):
			if (_mx == ea.getX() && _my == ea.getY())
				pick(ea, viewer);
			return true;
		default:
			return false;
	}
}

void mouseHandler::pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer) {
	osg::Node *scene = viewer->getSceneData();
	if (!scene) return;

	osg::Group *grandparent = 0;
	osgUtil::LineSegmentIntersector *picker;
	double x = ea.getXnormalized(), y = ea.getYnormalized();
	picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, x, y);
	picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE);
	osgUtil::IntersectionVisitor iv(picker);
	iv.setTraversalMask(IS_PICKABLE_MASK);
	viewer->getCamera()->accept(iv);

	if (picker->containsIntersections()) {
		// get node at first intersection
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
		osg::NodePath &nodePath = intersection.nodePath;

		// get robot node
		grandparent = (nodePath.size()>=3) ? dynamic_cast<osg::Group *>(nodePath[nodePath.size()-3]) : 0;

		// toggle HUD
		if (grandparent && (grandparent->getName() == "robot")) {
			osg::Geode *geode = dynamic_cast<osg::Geode *>(grandparent->getChild(0));
			geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
		}
	}
}

