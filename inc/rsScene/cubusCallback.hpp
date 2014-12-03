#ifndef RSSCENE_CUBUSCALLBACK_HPP_
#define RSSCENE_CUBUSCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>

class Cubus;

namespace rsScene {

class cubusCallback : public osg::NodeCallback {
	public:
		cubusCallback(Cubus*, osg::ShapeDrawable*);
		virtual void operator()(osg::Node*, osg::NodeVisitor*);
	private:
		Cubus *_robot;
		osg::ShapeDrawable *_led;
		int _count;
};

} // namespace rsScene

#endif // RSSCENE_CUBUSCALLBACK_HPP_

