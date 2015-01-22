#ifndef RSSCENE_MOUSEHANDLER_HPP_
#define RSSCENE_MOUSEHANDLER_HPP_

#include <osg/Geode>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgUtil/IntersectionVisitor>
#include <osgViewer/Viewer>

extern osg::Node::NodeMask NOT_VISIBLE_MASK;
extern osg::Node::NodeMask RECEIVES_SHADOW_MASK;
extern osg::Node::NodeMask CASTS_SHADOW_MASK;
extern osg::Node::NodeMask IS_PICKABLE_MASK;
extern osg::Node::NodeMask VISIBLE_MASK;

namespace rsScene {

	// forward declare
	class Scene;

	// classes
	class mouseHandler : public osgGA::GUIEventHandler {
		// public functions
		public:
			mouseHandler(rsScene::Scene*);
			virtual ~mouseHandler(void) {};

			// from GUIEventHandler
			bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
			void pick(const osgGA::GUIEventAdapter&, osgViewer::Viewer*);

		// private data
		private:
			double _mx;				// mouse x position
			double _my;				// mouse y position
			rsScene::Scene *_scene;	// scene
	};

} // namespace rsScene

#endif // RSSCENE_MOUSEHANDLER_HPP_

