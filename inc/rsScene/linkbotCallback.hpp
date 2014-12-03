#ifndef RSSCENE_LINKBOTCALLBACK_HPP_
#define RSSCENE_LINKBOTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

class CLinkbotT;

namespace rsScene {

class linkbotCallback : public osg::NodeCallback {
	public:
		linkbotCallback(CLinkbotT*, osg::ShapeDrawable*);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		CLinkbotT *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

} // namespace rsScene

#endif // RSSCENE_LINKBOTCALLBACK_HPP_

