#ifndef RSCALLBACK_MINDSTORMSCALLBACK_HPP_
#define RSCALLBACK_MINDSTORMSCALLBACK_HPP_

#include <osg/Node>
#include <osg/NodeVisitor>

#include <rsSim/Mindstorms.hpp>

namespace rsCallback {

	class mindstormsCallback : public osg::NodeCallback {
		// public functions
		public:
			mindstormsCallback(rsSim::Mindstorms*, rsSim::BodyList&, bool);
			virtual ~mindstormsCallback(void) {};

			virtual void operator()(osg::Node*, osg::NodeVisitor*);

		// private data
		private:
			bool _units;
			int _count;
			rsSim::Mindstorms *_robot;
			rsSim::BodyList _bodies;
	};

} // namespace rsCallback

#endif // RSCALLBACK_MINDSTORMSCALLBACK_HPP_

