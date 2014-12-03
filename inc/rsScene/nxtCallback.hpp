#ifndef RSSCENE_NXTCALLBACK_HPP_
#define RSSCENE_NXTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>

class CNXT;

namespace rsScene {

class nxtCallback : public osg::NodeCallback {
	public:
		nxtCallback(CNXT*, osg::ShapeDrawable*);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		CNXT *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

} // namespace rsScene

#endif // RSSCENE_NXTCALLBACK_HPP_

