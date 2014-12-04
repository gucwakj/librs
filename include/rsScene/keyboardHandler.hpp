#ifndef RSSCENE_KEYBOARD_HANDLER_HPP_
#define RSSCENE_KEYBOARD_HANDLER_HPP_

#include <osg/Billboard>
#include <osg/PositionAttitudeTransform>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgShadow/ShadowedScene>
#include <osgText/Text>
#include <osgViewer/Viewer>

extern osg::Node::NodeMask NOT_VISIBLE_MASK;
extern osg::Node::NodeMask RECEIVES_SHADOW_MASK;
extern osg::Node::NodeMask CASTS_SHADOW_MASK;
extern osg::Node::NodeMask IS_PICKABLE_MASK;
extern osg::Node::NodeMask VISIBLE_MASK;

namespace rsScene {

class keyboardHandler : public osgGA::GUIEventHandler {
	public:
		keyboardHandler(osgText::Text*);
		virtual bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
		virtual void accept(osgGA::GUIEventHandlerVisitor&);
	private:
		osgText::Text *_text;
};

} // namespace rsScene

#endif // RSSCENE_KEYBOARD_HANDLER_HPP_

