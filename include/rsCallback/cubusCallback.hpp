#ifndef RSCALLBACK_CUBUSCALLBACK_HPP_
#define RSCALLBACK_CUBUSCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

class Cubus;

namespace rsCallback {

	class cubusCallback : public osg::NodeCallback {
		// public functions
		public:
			cubusCallback(Cubus*, osg::ShapeDrawable*);
			virtual ~cubusCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			Cubus *_robot;
			osg::ShapeDrawable *_led;
			int _count;
	};

} // namespace rsCallback

#endif // RSCALLBACK_CUBUSCALLBACK_HPP_

