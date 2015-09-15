#include <osgFX/Scribe>

#include <rsScene/KeyboardHandler>

using namespace rsScene;

bool KeyboardHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	if (!viewer) return false;

	osg::Group *root = dynamic_cast<osg::Group *>(viewer->getSceneData());
	osgShadow::ShadowedScene *shadow = dynamic_cast<osgShadow::ShadowedScene *>(root->getChild(0));
	osg::Group *background = dynamic_cast<osg::Group *>(shadow->getChild(0));

	switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN: {
			this->keyPressed(ea.getKey());
			switch (ea.getKey()) {
				case '1': {
					osg::Vec3f eye = osg::Vec3f(0.6, -0.8, 0.5);
					osg::Vec3f center = osg::Vec3f(0.1, 0.3, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case '2': {
					osg::Vec3f eye = osg::Vec3f(0, 0, 5);
					osg::Vec3f center = osg::Vec3f(0, 0, 0);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case 'c': {
					// find highlighted item
					osg::Group *test = NULL;
					osg::BoundingSphere bs;
					for (unsigned int i = 0; i < shadow->getNumChildren(); i++) {
						test = dynamic_cast<osg::Group *>(shadow->getChild(i));
						// get robot node
						if (test && (!test->getName().compare(0, 5, "robot"))) {
							if (dynamic_cast<osgFX::Scribe *>(test->getChild(2)->asTransform()->getChild(0))) {
								bs = test->getBound();
								break;
							}
						}
						// get obstacle node
						else if (test && !test->getName().compare(0, 8, "obstacle")) {
							if (dynamic_cast<osgFX::Scribe *>(test->getChild(0)->asTransform()->getChild(0))) {
								bs = test->getBound();
								break;
							}
						}
						// get marker node
						else if (test && !test->getName().compare(0, 6, "marker")) {
							if (dynamic_cast<osgFX::Scribe *>(test->getChild(0)->asTransform()->getChild(0))) {
								bs = test->getBound();
								break;
							}
						}
					}
					// reposition camera
					osg::Vec3f eye = osg::Vec3f(0.6, -0.8, 0.5);
					osg::Vec3f center = osg::Vec3f(bs.center()[0], bs.center()[1], bs.center()[2]);
					osg::Vec3f up = osg::Vec3f(0, 0, 1);
					viewer->getCameraManipulator()->setHomePosition(eye, center, up);
					viewer->getCameraManipulator()->home(ea, aa);
					return true;
				}
				case 'n': {
					for (unsigned int i = 0; i < background->getNumChildren(); i++) {
						if (!background->getChild(i)->getName().compare("xnumbering")) {
							background->getChild(i)->setNodeMask(background->getChild(i)->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK);
							break;
						}
					}
					for (unsigned int i = 0; i < background->getNumChildren(); i++) {
						if (!background->getChild(i)->getName().compare("ynumbering")) {
							background->getChild(i)->setNodeMask(background->getChild(i)->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK);
							break;
						}
					}
					return true;
				}
				case 'r': {
					for (unsigned int i = 0; i < shadow->getNumChildren(); i++) {
						if (!shadow->getChild(i)->getName().compare(0, 5, "robot")) {
							// enable tracing
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) { geode->setNodeMask(VISIBLE_MASK); }
							// toggle robot visibility
							for (unsigned int j = 2; j < shadow->getChild(i)->asGroup()->getNumChildren(); j++) {
								osg::PositionAttitudeTransform *pat;
								pat = dynamic_cast<osg::PositionAttitudeTransform *>(shadow->getChild(i)->asGroup()->getChild(j));
								pat->setNodeMask((pat->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
							}
						}
					}
					return true;
				}
				case 't': {
					for (unsigned int i = 0; i < shadow->getNumChildren(); i++) {
						if (!shadow->getChild(i)->getName().compare(0, 5, "robot")) {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements())
								geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
						}
					}
					return true;
				}
				default:
					return true;
			}
		}
		default:
			return false;
	}
}

