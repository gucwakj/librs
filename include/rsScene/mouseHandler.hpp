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

	class mouseHandler : public osgGA::GUIEventHandler {
		// public functions
		public:
			mouseHandler(void);
			virtual ~mouseHandler(void) {};

			bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
			void pick(const osgGA::GUIEventAdapter&, osgViewer::Viewer*);

		// private data
		private:
			double _mx, _my;
	};

} // namespace rsScene

#endif // RSSCENE_MOUSEHANDLER_HPP_

