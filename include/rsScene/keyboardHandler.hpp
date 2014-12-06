#ifndef RSSCENE_KEYBOARDHANDLER_HPP_
#define RSSCENE_KEYBOARDHANDLER_HPP_

#include <osg/Billboard>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgShadow/ShadowedScene>
#include <osgViewer/Viewer>

extern osg::Node::NodeMask NOT_VISIBLE_MASK;
extern osg::Node::NodeMask RECEIVES_SHADOW_MASK;
extern osg::Node::NodeMask CASTS_SHADOW_MASK;
extern osg::Node::NodeMask IS_PICKABLE_MASK;
extern osg::Node::NodeMask VISIBLE_MASK;

namespace rsScene {

	class keyboardHandler : public osgGA::GUIEventHandler {
		// public functions
		public:
			keyboardHandler(void) {};
			virtual ~keyboardHandler(void) {};

			virtual bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
			virtual void accept(osgGA::GUIEventHandlerVisitor&);

		// virtual functions for inherited classes
		protected:
			virtual void keyPressed(int) {};
	};

} // namespace rsScene

#endif // RSSCENE_KEYBOARDHANDLER_HPP_

