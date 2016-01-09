#include <osgText/Text>

#include <rsScene/MouseHandler>
#include <rsScene/Scene>

using namespace rsScene;

MouseHandler::MouseHandler(rsScene::Scene *scene) {
	// default mouse position
	_mx = 0.0;
	_my = 0.0;

	// scene to which this handler is attached
	_scene = scene;

	// give it a name
	this->setName("mouse");
}

/**********************************************************
	public functions
 **********************************************************/
bool MouseHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
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

int MouseHandler::pick(const osgGA::GUIEventAdapter &ea, osgViewer::Viewer *viewer) {
	osg::Node *scene = viewer->getSceneData();
	if (!scene) return -1;

	// run intersector from mouse through scene graph
	double x = ea.getXnormalized(), y = ea.getYnormalized();
	osg::ref_ptr<osgUtil::LineSegmentIntersector> picker;
	picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::PROJECTION, x, y);
	picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_ONE_PER_DRAWABLE);
	osgUtil::IntersectionVisitor iv(picker);
	iv.setTraversalMask(IS_PICKABLE_MASK);
	viewer->getCamera()->accept(iv);

	// get robots in intersections
	if (picker->containsIntersections()) {
		// get node at first intersection
		osgUtil::LineSegmentIntersector::Intersection intersection = picker->getFirstIntersection();
		osg::NodePath &nodePath = intersection.nodePath;

		// find nodes of intersection
		osg::Group *test = NULL;
		for (unsigned int i = 0; i < nodePath.size() - 2; i++) {
			test = dynamic_cast<osg::Group *>(nodePath[i]);
			// get preconfig node
			if (test && !test->getName().compare(0, 3, "pre")) {
				int id = atoi(&(test->getName()[3]));
				_scene->addHighlight(id, true, false, true);
				_scene->toggleLabel(test, dynamic_cast<osg::Node *>(nodePath[i + 3]));
				return id;
			}
			// get robot node
			else if (test && !test->getName().compare(0, 5, "robot")) {
				int id = atoi(&(test->getName()[5]));
				_scene->addHighlight(id, true, false, true);
				_scene->toggleLabel(test, dynamic_cast<osg::Node *>(nodePath[i + 2]));
				return id;
			}
			// get ground node
			else if (test && !test->getName().compare(0, 6, "ground")) {
				int id = atoi(&(test->getName()[6]));
				_scene->addHighlight(id, false, false, true);
				_scene->toggleLabel(test, dynamic_cast<osg::Node *>(nodePath[i + 2]));
				return id;
			}
		}
	}
	return -1;
}

