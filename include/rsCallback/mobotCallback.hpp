#ifndef RSCALLBACK_MOBOTCALLBACK_HPP_
#define RSCALLBACK_MOBOTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

class CMobot;

namespace rsCallback {

	class mobotCallback : public osg::NodeCallback {
		// public functions
		public:
			mobotCallback(CMobot*, osg::ShapeDrawable*);
			virtual ~mobotCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			CMobot *_robot;
			osg::ShapeDrawable *_led;
			int _count;
	};

} // namespace rsCallback

#endif // RSCALLBACK_MOBOTCALLBACK_HPP_
