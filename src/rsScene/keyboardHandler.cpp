#include <rsScene/keyboardHandler.hpp>

using namespace rsScene;

void keyboardHandler::accept(osgGA::GUIEventHandlerVisitor &v) {
	v.visit(*this);
}

bool keyboardHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
	osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);
	if (!viewer) return false;

	osg::Group *root = dynamic_cast<osg::Group *>(viewer->getSceneData());
	osgShadow::ShadowedScene *shadow = dynamic_cast<osgShadow::ShadowedScene *>(root->getChild(0));

	switch (ea.getEventType()) {
		case osgGA::GUIEventAdapter::KEYDOWN:
			this->keyPressed(ea.getKey());
			switch (ea.getKey()) {
				case '1': {
					osg::Vec3f eye = osg::Vec3f(0.7, -0.7, 0.55);
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
				case 'n': {
					osg::Billboard *billboard = dynamic_cast<osg::Billboard *>(root->getChild(3));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					billboard = dynamic_cast<osg::Billboard *>(root->getChild(4));
					billboard->setNodeMask((billboard->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
					return true;
				}
				case 'r': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements()) { geode->setNodeMask(VISIBLE_MASK); }
							for (int j = 2; j < (int)(shadow->getChild(i)->asGroup()->getNumChildren()); j++) {
								osg::PositionAttitudeTransform *pat;
								pat = dynamic_cast<osg::PositionAttitudeTransform *>(shadow->getChild(i)->asGroup()->getChild(j));
								pat->setNodeMask((pat->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
							}
						}
					}
					return true;
				}
				case 't': {
					for (int i = 0; i < (int)(shadow->getNumChildren()); i++) {
						if (shadow->getChild(i)->getName() == "robot") {
							osg::Geode *geode = dynamic_cast<osg::Geode *>(shadow->getChild(i)->asGroup()->getChild(1));
							osg::Geometry *draw = dynamic_cast<osg::Geometry *>(geode->getDrawable(0)->asGeometry());
							osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(draw->getVertexArray());
							if (vertices->getNumElements())
								geode->setNodeMask((geode->getNodeMask() ? NOT_VISIBLE_MASK : VISIBLE_MASK));
						}
					}
					return true;
				}
				default: {
					return true;
				}
			}
		default:
			return false;
	}
}

