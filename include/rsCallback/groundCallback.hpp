#ifndef RSCALLBACK_GROUNDCALLBACK_HPP_
#define RSCALLBACK_GROUNDCALLBACK_HPP_

#include <osg/Group>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>

#include <rsSim/sim.hpp>

namespace rsCallback {

	class groundCallback : public osg::NodeCallback {
		// public functions
		public:
			groundCallback(rsSim::Ground*);
			virtual ~groundCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			rsSim::Ground *_ground;
	};

} // namespace rsCallback

#endif // RSCALLBACK_GROUNDCALLBACK_HPP_

