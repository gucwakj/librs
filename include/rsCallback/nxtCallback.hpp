#ifndef RSCALLBACK_NXTCALLBACK_HPP_
#define RSCALLBACK_NXTCALLBACK_HPP_

#include <osg/Geode>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>

class CNXT;

namespace rsCallback {

	class nxtCallback : public osg::NodeCallback {
		// public functions
		public:
			nxtCallback(CNXT*, osg::ShapeDrawable*);
			virtual ~nxtCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			CNXT *_robot;
			osg::ShapeDrawable *_led;
			int _count;
	};

} // namespace rsCallback

#endif // RSCALLBACK_NXTCALLBACK_HPP_

