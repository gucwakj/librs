#ifndef RSSCENE_MOBOTCALLBACK_HPP_
#define RSSCENE_MOBOTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>

class CMobot;

namespace rsScene {

class mobotCallback : public osg::NodeCallback {
	public:
		mobotCallback(CMobot*, osg::ShapeDrawable*);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		CMobot *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

} // namespace rsScene

#endif // RSSCENE_MOBOTCALLBACK_HPP_

