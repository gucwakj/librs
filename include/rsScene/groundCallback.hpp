#ifndef RSSCENE_GROUNDCALLBACK_HPP_
#define RSSCENE_GROUNDCALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>

namespace rsScene {

	class groundCallback : public osg::NodeCallback {
		// public functions
		public:
			groundCallback(Ground*);
			virtual ~groundCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			Ground *_ground;
	};

} // namespace rsScene

#endif // RSSCENE_GROUNDCALLBACK_HPP_

